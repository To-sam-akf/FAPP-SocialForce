#include "mapping_manager.h"
#include "social_force_predictor.h" // <--- 在 .cpp 文件中包含完整的头文件

namespace mot_mapping {

MappingRos::MappingRos(ros::NodeHandle &nh):nh(nh) {}

MappingRos::~MappingRos() {std::cout << "Exit Tracker" << std::endl;}

void MappingRos::init() {
  t_start = ros::Time::now();
  frame_num = 1;
  id = 0;

  Remaining_Points.reset(new Points);
  ikdtree_ptr.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));

  mp_ = std::make_unique<MapParam>();
  md_ = std::make_unique<MapData>();

  nh.param("dbscan/core_pts", mp_->core_pts, 4);
  nh.param("dbscan/tolerance", mp_->tolerance, 0.02);
  nh.param("dbscan/min_cluster", mp_->min_cluster, 20);
  nh.param("dbscan/max_cluster", mp_->max_cluster, 800);
  nh.param("detection/thresh_dist", mp_->thresh_dist, 0.08);
  nh.param("detection/thresh_var", mp_->thresh_var, 0.28);
  nh.param("lidar/range_x", mp_->Range[0], 15.0);
  nh.param("lidar/range_y", mp_->Range[1], 15.0);
  nh.param("lidar/range_z", mp_->Range[2], 1.0);
  nh.param("ros/odom_topic", mp_->odom_topic, string("/odom"));
  nh.param("ros/lidar_topic", mp_->lidar_topic, string("/livox/lidar"));

  cloudPub = nh.advertise<sensor_msgs::PointCloud2>("/dynamic_points", 10);
  mapPub = nh.advertise<sensor_msgs::PointCloud2>("/map_ros", 10);
  staticMapPub = nh.advertise<sensor_msgs::PointCloud2>("/static_map", 10);
  edgePub = nh.advertise<visualization_msgs::MarkerArray>("/box_edge", 1000);
  objectPosePub = nh.advertise<visualization_msgs::MarkerArray>("/object_pose", 10);
  statesPub = nh.advertise<obj_state_msgs::ObjectsStates>("/states", 10);

  // 在这里添加对 social_predictor_ 的初始化
  // 添加这一行
  // forcePub = nh.advertise<visualization_msgs::MarkerArray>("/social_forces", 10);
  forcePub = nh.advertise<visualization_msgs::MarkerArray>("/social_forces", 10);
  // --- 调试代码开始 ---
  if (forcePub) {
      ROS_INFO("Publisher for /social_forces created successfully.");
  } else {
      ROS_ERROR("Failed to create publisher for /social_forces!");
  }

  social_predictor_ = std::make_unique<SocialForcePredictor>();

  std::cout << "INIT!" << std::endl;

  cloud_sub_.reset(
      new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/map_generator/obj_cloud", 50));
  odom_sub_.reset(
      new message_filters::Subscriber<nav_msgs::Odometry>(nh, "/drone_0_visual_slam/odom", 25));
  sync_cloud_odom_.reset(new message_filters::Synchronizer<MappingRos::SyncPolicyCloudOdom>(
      MappingRos::SyncPolicyCloudOdom(100), *cloud_sub_, *odom_sub_));
  sync_cloud_odom_->registerCallback(boost::bind(&MappingRos::cloudOdomCallback, this, _1, _2));

  ekf_predict_timer_ = nh.createTimer(ros::Duration(0.02), &MappingRos::ekfPredictCallback, this);
  map_pub_timer_ = nh.createTimer(ros::Duration(0.05), &MappingRos::mapPubCallback, this);
}

// 新增可视化函数
void MappingRos::visualizeForces(const std::vector<ObjectState>& detections,
                                const std::vector<std::vector<ObjectState>>& neighbors) {
    visualization_msgs::MarkerArray force_markers;
    int marker_id = 0;
    
    for (int i = 0; i < detections.size(); ++i) {
        if (!isHuman(detections[i])) continue;  // 只为人类对象显示社会力
        
        ForceComponents components = social_predictor_->getForceComponents(
            detections[i], neighbors[i]);
        
        // 1. 驱动力 - 黄色箭头
        auto driving_marker = createForceArrow(
            detections[i].position, 
            components.driving_force * 2.0,  // 放大显示
            marker_id++, 
            "blue"
        );
        force_markers.markers.push_back(driving_marker);
        
        // 2. 每个排斥力 - 红色箭头
        for (const auto& repulsion : components.individual_repulsions) {
          // 如果排斥力非常小，就不要画了，避免 Rviz 里面全是杂乱的小点
            if (repulsion.first.norm() < 0.01) continue; 

            auto repulsive_marker = createForceArrow(
                detections[i].position,
                repulsion.first * 5.0,  // 排斥力通常较小，放大更多
                marker_id++,
                "red"
            );
            force_markers.markers.push_back(repulsive_marker);
        }
        
        // 3. 合成力 - 色箭头
        auto total_marker = createForceArrow(
            detections[i].position,
            components.total_force * 2.0,
            marker_id++,
            "green"
        );
        force_markers.markers.push_back(total_marker);
    }
    ROS_INFO("Attempting to publish %zu force markers.", force_markers.markers.size());
    forcePub.publish(force_markers);
}

visualization_msgs::Marker MappingRos::createForceArrow(const Eigen::Vector3d& start,
                                                       const Eigen::Vector3d& force,
                                                       int id,
                                                       const std::string& color) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    // ==================== 修复 1: 初始化四元数 ====================
    // 即使使用 points 定义箭头，Pose 也必须有效。w=1 代表无旋转。
    marker.pose.orientation.w = 1.0; 
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;

     // ==================== 修复 2: 检查力的大小 ====================
    // 如果力太小，直接返回一个空操作的 marker，或者不添加到 Array 中
    // 这里为了简单，如果太小我们让它极其微小但非零，或者依靠调用者的逻辑
    // 但为了保险，我们把起点和终点设置好
    // 起点
    marker.points.resize(2);
    marker.points[0].x = start.x();
    marker.points[0].y = start.y();
    marker.points[0].z = start.z() + 0.5;  // 稍微抬高避免与地面重叠
    
    // 终点
    marker.points[1].x = start.x() + force.x();
    marker.points[1].y = start.y() + force.y();
    marker.points[1].z = start.z() + 0.5 + force.z();
    
    // 箭头样式
    marker.scale.x = 0.2;  // 箭头杆粗细
    marker.scale.y = 0.20;   // 箭头头部宽度
    marker.scale.z = 1.0;   // 箭头头部长度
    
  // 颜色设置
    if (color == "yellow") {
        marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0;
    } else if (color == "red") {
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0;
    } else if (color == "blue") { // 你之前的代码写的是 "blue"
        marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0;
    } else if (color == "green") {
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
    }
    marker.color.a = 0.8;
    
    marker.lifetime = ros::Duration(0.1);  // 短暂显示避免积累
    
    return marker;
  }


float MappingRos::calc_dist(PointType p1, PointType p2) {
    float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    return d;
}

void MappingRos::generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths) {
    float &x_dist = box_lengths[0];
    float &y_dist = box_lengths[1];
    float &z_dist = box_lengths[2];

    boxpoint.vertex_min[0] = center_pt.x - x_dist;
    boxpoint.vertex_max[0] = center_pt.x + x_dist;
    boxpoint.vertex_min[1] = center_pt.y - y_dist;
    boxpoint.vertex_max[1] = center_pt.y + y_dist;
    boxpoint.vertex_min[2] = center_pt.z - z_dist;
    boxpoint.vertex_max[2] = center_pt.z + z_dist;
}

void MappingRos::pointsBodyToWorld(const PointsPtr p_b, PointsPtr p_w) {
  int num = p_b->points.size();
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    p << p_b->points[i].x, 
         p_b->points[i].y, 
         p_b->points[i].z;
    // p = md_->R * p + md_->T;
    if ((p - md_->T).head(2).norm() < 5.0) {
        PointType pt;
        pt.x = p[0];
        pt.y = p[1];
        pt.z = p[2];

        p_w->points.push_back(pt);
    }
  }
}

void MappingRos::pointsWorldToBody(const PointsPtr p_w, PointsPtr p_b, Eigen::Vector3d T) {
  int num = p_w->points.size();
  Eigen::Vector3d p;
  for (int i = 0; i < num; ++i) {
    p << p_w->points[i].x, 
         p_w->points[i].y, 
         p_w->points[i].z;
    p = (p - md_->T);

    PointType pt;
    pt.x = p[0];
    pt.y = p[1];
    pt.z = p[2];
    p_b->points.push_back(pt);
  }
}


bool MappingRos::isHuman(const ObjectState& object) const {
  // // --- 调试代码开始 ---
  // double avg_width = (object.size.x() + object.size.y()) / 2.0;
  // double height_width_ratio = (avg_width > 0.01) ? (object.size.z() / avg_width) : 0.0;
  
  // // 使用 ROS_INFO 打印关键信息。%d代表整数, %.2f代表保留两位小数的浮点数。
  // ROS_INFO("Checking isHuman for object ID [%d]: H=%.2f, W_avg=%.2f, Ratio=%.2f",
  //          object.id, object.size.z(), avg_width, height_width_ratio);
  // // --- 调试代码结束 ---

  // if (object.size.z() > 1.2 * avg_width && object.size.z() > 0.8 && object.size.z() < 2.2) {
  //   // 如果判断为真，再打印一条确认信息
  //   ROS_INFO("===> Object [%d] CLASSIFIED AS HUMAN!", object.id);
  //   return true;
  // }
  
  // return false;
  return true;
}

std::vector<ObjectState> MappingRos::getAllNearbyObjects(const Eigen::Vector3d& position) const {
  std::vector<ObjectState> neighbors;
  const double social_radius = 5.0; // 定义一个“社交”范围，比如5米

  for (const auto& obj : detections) {
    // 计算距离，但不包括物体自身
    double distance = (obj.position - position).norm();
    if (distance > 0.01 && distance < social_radius) {
      neighbors.push_back(obj);
    }
  }

  return neighbors;
}


void MappingRos::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& msg, 
                                   const nav_msgs::OdometryConstPtr& odom) {
  PointsPtr latest_cloud(new Points);
  pcl::fromROSMsg(*msg, *latest_cloud);

  md_->T << odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z;
       
  md_->R = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                              odom->pose.pose.orientation.y, odom->pose.pose.orientation.z).toRotationMatrix();

  std::chrono::high_resolution_clock::time_point tic = std::chrono::high_resolution_clock::now();
  double compTime;
  // clear screen
  printf("\033[2J");
  printf("\033[1;1H");
  
  // Pointcloud Filter
  PointsPtr cloud_filtered(new Points);
  vox.setInputCloud(latest_cloud);
  vox.setLeafSize(0.1, 0.1, 0.1);
  vox.filter(*cloud_filtered);

  PointsPtr cloud_world(new Points);
  PointsPtr nonfilter_pts(new Points);
  PointsPtr PointToAdd(new Points);
  PointsPtr ClusterPoints(new Points);
  pointsBodyToWorld(cloud_filtered, cloud_world);
  pointsBodyToWorld(latest_cloud, nonfilter_pts);

  buffer.push_back(nonfilter_pts);
  if (buffer.size() > 1) {
    buffer.pop_front();
  }
  for (int i = 0; i < buffer.size(); ++i) {
    *ClusterPoints += *buffer[i];
  }
  vox.setInputCloud(ClusterPoints);
  vox.setLeafSize(0.05, 0.05, 0.05);
  vox.filter(*ClusterPoints);

  Remaining_Points = ClusterPoints;
  
  PointsPtr All_Points(new Points);

  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Filter Time Cost (ms)： " << compTime <<std::endl;
  tic = std::chrono::high_resolution_clock::now();

  std::cout << "Input Size:" << ClusterPoints->size() << std::endl;
  
  sensor_msgs::PointCloud2 map_ros;
  PointsPtr OutputPoints(new Points);
  vox.setInputCloud(ClusterPoints);
  vox.setLeafSize(0.05, 0.05, 0.05);
  vox.filter(*OutputPoints);

  pcl::toROSMsg(*OutputPoints, map_ros);
  map_ros.header.frame_id = "world";
  mapPub.publish(map_ros);

  // DBSCAN Cluster
  std::vector<pcl::PointIndices> cluster_indices;
  dbscan.setCorePointMinPts(mp_->core_pts);
  dbscan.setClusterTolerance(mp_->tolerance);
  dbscan.setMinClusterSize(mp_->min_cluster);
  dbscan.setMaxClusterSize(mp_->max_cluster);
  dbscan.setInputCloud(ClusterPoints);
  dbscan.setSearchMethod();
  dbscan.extractNano(cluster_indices);

  std::cout << "Cluster size: " << cluster_indices.size() << std::endl;
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Cluster Time Cost (ms)： " << compTime <<std::endl;
  tic = std::chrono::high_resolution_clock::now();

  if (frame_num < 2){
    previous_points.push_back(cloud_world);
    frame_num++;
    return;
  }

  int pt_size = previous_points[0]->points.size();
  //ikd-tree
  if (ikdtree_ptr->Root_Node == nullptr) {
    ikdtree_ptr->Build(previous_points[0]->points);
    previous_points.push_back(cloud_world);
    previous_points.pop_front();
  }
  else {
    PointVector PointNoNeedDownsample;
    bool need_add = true;

    for (int i = 0; i < pt_size; ++i) {
      PointType mid_point; 
      mid_point.x = floor(previous_points[0]->points[i].x/0.1)*0.1 + 0.5 * 0.1;
      mid_point.y = floor(previous_points[0]->points[i].y/0.1)*0.1 + 0.5 * 0.1;
      mid_point.z = floor(previous_points[0]->points[i].z/0.1)*0.1 + 0.5 * 0.1;
      // float dist  = calc_dist(cloud_filtered->points[i],mid_point);
      PointVector points_near;
      vector<float> pointSearchSqDis(5);
      ikdtree_ptr->Nearest_Search(mid_point, 1, points_near, pointSearchSqDis);

      if (fabs(points_near[0].x - mid_point.x) > 0.5 * 0.1 || 
          fabs(points_near[0].y - mid_point.y) > 0.5 * 0.1 || 
          fabs(points_near[0].z - mid_point.z) > 0.5 * 0.1) {
        PointNoNeedDownsample.push_back(previous_points[0]->points[i]);
        PointToAdd->points.push_back(previous_points[0]->points[i]);
      }
    }
    // int add_point_size = ikdtree_ptr->Add_Points(PointToAdd, true);
    ikdtree_ptr->Add_Points(PointNoNeedDownsample, false); 
    input_point.push_back(PointToAdd);
    if (input_point.size() > 25) {
      PointVector PointDelete;
      for (auto& delet_pt: input_point[0]->points) {
        PointDelete.push_back(delet_pt);
      }
      ikdtree_ptr->Delete_Points(PointDelete);
      input_point.pop_front();
    }

    previous_points.push_back(cloud_world);
    previous_points.pop_front();
  }
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "ikd-Tree Time Cost (ms)： " << compTime << std::endl;
  tic = std::chrono::high_resolution_clock::now();
  std::cout << "ikd-Tree Size: " << ikdtree_ptr->size() << std::endl;
  // ikdtree_ptr->flatten(ikdtree_ptr->Root_Node, ikdtree_ptr->PCL_Storage, NOT_RECORD);
  // All_Points->points = ikdtree_ptr->PCL_Storage;
  // All_Points->width = All_Points->points.size();
  // All_Points->height = 1;
  // All_Points->is_dense = true;

  // sensor_msgs::PointCloud2 previous_cloud;
  // pcl::toROSMsg(*All_Points, previous_cloud);
  // previous_cloud.header.frame_id = "world";
  // cloudPub.publish(previous_cloud);

  PointsPtr dynamic_points(new Points);

  int k = 0;
  detections.clear();
  deleted_indices.clear();
  for (auto& getIndices: cluster_indices) {
    PointsPtr cluster(new Points);
    PointVector PointDelete;
    double avg_dist = 0;
    Eigen::Vector3d cluster_center;
    cluster_center.setZero();
    std::vector<double> avg_buffer;
    for (auto& index : getIndices.indices) {
      cluster->points.push_back(ClusterPoints->points[index]);
      PointVector points_near;
      vector<float> pointSearchSqDis(5);
      ikdtree_ptr->Nearest_Search(ClusterPoints->points[index], 3, points_near, pointSearchSqDis);
      avg_dist += sqrt(pointSearchSqDis[0]);
      avg_buffer.push_back(sqrt(pointSearchSqDis[0]));
      cluster_center += Eigen::Vector3d(ClusterPoints->points[index].x, ClusterPoints->points[index].y, ClusterPoints->points[index].z);
    }

    avg_dist = avg_dist/getIndices.indices.size();
    cluster_center = cluster_center/getIndices.indices.size();
    double var_dist = 0;
    for (auto& dist : avg_buffer) {
      var_dist += (dist - avg_dist) * (dist - avg_dist) / (avg_dist * avg_dist);
    }
    var_dist = var_dist/getIndices.indices.size();
    
    bool on_track = false;
    for (int i = 0; i < trackers.size(); ++i) {
      Eigen::Vector3d tracker_center = trackers[i]->pos();
      double dist = sqrt((cluster_center(0) - tracker_center(0)) * (cluster_center(0) - tracker_center(0)) + 
                         (cluster_center(1) - tracker_center(1)) * (cluster_center(1) - tracker_center(1)) + 
                         (cluster_center(2) - tracker_center(2)) * (cluster_center(2) - tracker_center(2)));
      if (dist < 0.5 && trackers[i]->age > 3 && trackers[i]->vel().norm() > 0.2) {
        on_track = true;
        break;
      }
    }

    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
    pcl::PointXYZ minPt, maxPt;
	  pcl::getMinMax3D(*cluster, minPt, maxPt);

    PointType center_pt;
    center_pt.x = (maxPt.x + minPt.x)/2;
    center_pt.y = (maxPt.y + minPt.y)/2;
    center_pt.z = (maxPt.z + minPt.z)/2;

    double radius_xy = (center_pt.x - md_->T[0]) * (center_pt.x - md_->T[0]) + 
                       (center_pt.y - md_->T[1]) * (center_pt.y - md_->T[1]);
    double radius_z = (center_pt.z - md_->T[2]) * (center_pt.z - md_->T[2]);
    

    if ((avg_dist > mp_->thresh_dist && avg_dist < 5.0 && var_dist < mp_->thresh_var) || on_track) {
      // std::cout << "on_track: " << on_track << std::endl;
      // std::cout << "var_dist: " << var_dist << std::endl;
      // std::cout << "avg_dist: " << avg_dist << std::endl;
      ObjectState state;
      state.id = k;
      state.position = cluster_center;
      state.size << maxPt.x - minPt.x, maxPt.y - minPt.y, maxPt.z - minPt.z;
      detections.push_back(state);
      deleted_indices.push_back(getIndices);
      for (auto& index : getIndices.indices) {
        dynamic_points->points.push_back(ClusterPoints->points[index]);
      }
      // BoxPointType box;
      // box.vertex_min[0] = minPt.x-0.2; box.vertex_min[1] = minPt.y-0.2; box.vertex_min[2] = minPt.z-0.2;
      // box.vertex_max[0] = maxPt.x+0.2; box.vertex_max[1] = maxPt.y+0.2; box.vertex_max[2] = maxPt.z+0.2;
      // delete_boxes.push_back(box);
    }
  }

  sensor_msgs::PointCloud2 dynamic_pts;
  pcl::toROSMsg(*dynamic_points, dynamic_pts);
  dynamic_pts.header.frame_id = "world";
  cloudPub.publish(dynamic_pts);

  // Tracking & EKF
  if(trackers.size() == 0) {
    for(int i = 0; i < detections.size(); ++i) {
      std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(0.02);
      ekfPtr->reset(detections[i].position, id);
      id++;
      trackers.push_back(ekfPtr);
      previous_p.push_back(detections[i].position);
      Eigen::Vector3d v0(0,0,0);
      previous_v.push_back(v0);
    }
    return;
  }


  obj_state_msgs::ObjectsStates states;
  std::vector<std::pair<int, int>> matchedPairs;

  for (int i = 0; i < detections.size(); ++i) {
    double min_dist = 1000000;
    int min_index = -1;
    Eigen::Vector3d det_pos = detections[i].position;
    for (int j = 0; j < trackers.size(); ++j) {
      trackers[j]->age = trackers[j]->age + 1;
      Eigen::Vector3d track_pos = trackers[j]->pos();
      double dist = (det_pos - track_pos).norm();
      if (dist < min_dist) {
        min_dist = dist;
        min_index = j;
      }
    }

    if (min_dist < 0.8) {
      // std::cout << "id:" << trackers[min_index]->id << std::endl;
      // 替换第370-371行
    if (isHuman(detections[i])) {
        // 使用社会力模型
        Eigen::Vector3d predicted_vel = social_predictor_->predictTrajectory(
            detections[i], getAllNearbyObjects(detections[i].position));
        trackers[min_index]->update(detections[i].position, predicted_vel);
    } else {
        // 保持原有线性预测
        // Eigen::Vector3d vel_detect = (detections[i].position - previous_p[min_index]) / 0.02;
        // trackers[min_index]->update(detections[i].position, 0.5*vel_detect + 0.5*previous_v[min_index]);
        cout<<"error"<<endl;
    }

      matchedPairs.push_back(std::make_pair(i, min_index));

      ObjectState state;
      state.position = trackers[min_index]->pos();
      state.velocity = trackers[min_index]->vel();
      obj_state_msgs::State statemsg;
      statemsg.header.stamp = ros::Time::now();
      statemsg.position.x = state.position[0]; statemsg.position.y = state.position[1]; statemsg.position.z = state.position[2];
      statemsg.velocity.x = state.velocity[0]; statemsg.velocity.y = state.velocity[1]; statemsg.velocity.z = state.velocity[2];
      statemsg.size.x = 0; statemsg.size.y = 0; statemsg.size.z = 0;
      states.states.push_back(statemsg);

    } else {
      std::shared_ptr<Ekf> ekfPtr = std::make_shared<Ekf>(0.02);
      ekfPtr->reset(detections[i].position, id);
      id++;
      trackers.push_back(ekfPtr);
    }
  }
  statesPub.publish(states);
  // 添加调试输出
  std::cout << "=== Force Visualization Debug ===" << std::endl;
  std::cout << "About to publish forces..." << std::endl;

  // 新增：可视化社会力
  std::vector<std::vector<ObjectState>> all_neighbors;
  for (const auto& detection : detections) {
      all_neighbors.push_back(getAllNearbyObjects(detection.position));
  }
  visualizeForces(detections, all_neighbors);

  visualizeFunction(matchedPairs);

  for (auto it = trackers.begin(); it != trackers.end();) {
      // std::cout << "dt:" << (*it)->age - (*it)->update_num << std::endl;
      if ((*it)->age - (*it)->update_num > 20)
        it = trackers.erase(it);
      else 
        it++;
  }    

  previous_p.clear();
  previous_v.clear();
  for (int i = 0; i < trackers.size(); ++i ) {
    previous_p.push_back(trackers[i]->pos());
    previous_v.push_back(trackers[i]->vel());
  }
  compTime = std::chrono::duration_cast<std::chrono::microseconds>
                    (std::chrono::high_resolution_clock::now() - tic).count() * 1.0e-3;
  std::cout << "Tracking Time Cost (ms)： " << compTime <<std::endl;
}

void MappingRos::ekfPredictCallback(const ros::TimerEvent& e) {
  if (trackers.size() == 0)
    return;

  for (int i = 0; i < trackers.size(); ++i) {
    double update_dt = (ros::Time::now() - trackers[i]->last_update_stamp_).toSec();
    trackers[i]->predict();
  }
}

void MappingRos::mapPubCallback(const ros::TimerEvent& e) {
  if (Remaining_Points->points.size() == 0)
    return;
  
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // KD_TREE<PointType>::Ptr delete_tree;
  // delete_tree.reset(new KD_TREE<PointType>(0.3, 0.6, 0.05));
  // delete_tree->Build(Remaining_Points->points);
  // delete_tree->Delete_Point_Boxes(delete_boxes);

  // Points static_pts;
  // delete_tree->flatten(delete_tree->Root_Node, delete_tree->PCL_Storage, NOT_RECORD);
  // static_pts.points = delete_tree->PCL_Storage;
  // static_pts.width = static_pts.points.size();
  // static_pts.height = 1;
  // static_pts.is_dense = true;


  // delete_boxes.clear();
  // sensor_msgs::PointCloud2 map_static;
  // pcl::toROSMsg(static_pts, map_static);
  // map_static.header.frame_id = "world";
  // staticMapPub.publish(map_static);
}


double MappingRos::iou(ObjectState state1, ObjectState state2) {
  double distance = (state1.position - state2.position).norm();
  double iou = atan(distance)*2/M_PI;
  return iou;
}

void MappingRos::visualizeFunction(const std::vector<std::pair<int, int>> pairs) {
  visualization_msgs::MarkerArray poses;
  visualization_msgs::MarkerArray boxes;

  for (auto pair : pairs) {
    int detectionIndex = pair.first;
    int trackerIndex = pair.second;
    pcl::PointXYZ minPt, maxPt;
    // if (detections[detectionIndex].position[1] - detections[detectionIndex].size[1]/2 > 1.7 ||
    //     detections[detectionIndex].position[1] < -2.0 ||
    //     detections[detectionIndex].position[2] > 1.8)
    //   continue;
    minPt.x = detections[detectionIndex].position[0] - detections[detectionIndex].size[0]/2;
    minPt.y = detections[detectionIndex].position[1] - detections[detectionIndex].size[1]/2;
    minPt.z = detections[detectionIndex].position[2] - detections[detectionIndex].size[2]/2;
    maxPt.x = detections[detectionIndex].position[0] + detections[detectionIndex].size[0]/2;
    maxPt.y = detections[detectionIndex].position[1] + detections[detectionIndex].size[1]/2;
    maxPt.z = detections[detectionIndex].position[2] + detections[detectionIndex].size[2]/2;
    visualization_msgs::Marker edgeMarker;
    edgeMarker.id = detectionIndex;
    edgeMarker.header.stamp = ros::Time::now();
    edgeMarker.header.frame_id = "world";
    edgeMarker.pose.orientation.w = 1.00;
    edgeMarker.lifetime = ros::Duration(0.1);
    edgeMarker.type = visualization_msgs::Marker::LINE_STRIP;
    edgeMarker.action = visualization_msgs::Marker::ADD;
    edgeMarker.ns = "edge";
    edgeMarker.color.r = 1.00;
    edgeMarker.color.g = 0.50;
    edgeMarker.color.b = 0.00;
    edgeMarker.color.a = 0.80;
    edgeMarker.scale.x = 0.1;
    geometry_msgs::Point point[8];
    point[0].x = minPt.x; point[0].y = maxPt.y; point[0].z = maxPt.z;
    point[1].x = minPt.x; point[1].y = minPt.y; point[1].z = maxPt.z;
    point[2].x = minPt.x; point[2].y = minPt.y; point[2].z = minPt.z;
    point[3].x = minPt.x; point[3].y = maxPt.y; point[3].z = minPt.z;
    point[4].x = maxPt.x; point[4].y = maxPt.y; point[4].z = minPt.z;
    point[5].x = maxPt.x; point[5].y = minPt.y; point[5].z = minPt.z;
    point[6].x = maxPt.x; point[6].y = minPt.y; point[6].z = maxPt.z;
    point[7].x = maxPt.x; point[7].y = maxPt.y; point[7].z = maxPt.z;
    for (int l = 0; l < 8; l++) {
      edgeMarker.points.push_back(point[l]);
    }
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[3]);
    edgeMarker.points.push_back(point[2]);
    edgeMarker.points.push_back(point[5]);
    edgeMarker.points.push_back(point[6]);
    edgeMarker.points.push_back(point[1]);
    edgeMarker.points.push_back(point[0]);
    edgeMarker.points.push_back(point[7]);
    edgeMarker.points.push_back(point[4]);
    boxes.markers.push_back(edgeMarker); 

    visualization_msgs::Marker poseMarker;
    ObjectState state;
    state.position = trackers[trackerIndex]->pos();
    state.velocity = trackers[trackerIndex]->vel();
    poseMarker.id = trackerIndex+100;
    poseMarker.header.stamp = ros::Time::now();
    poseMarker.header.frame_id = "world";
    poseMarker.lifetime = ros::Duration(0.1);
    poseMarker.type = visualization_msgs::Marker::ARROW;
    poseMarker.action = visualization_msgs::Marker::ADD;
    poseMarker.ns = "objectpose";
    poseMarker.color.r = 0.00;
    poseMarker.color.g = 1.00;
    poseMarker.color.b = 0.00;
    poseMarker.color.a = 1.00;
    poseMarker.scale.x = 0.10;
    poseMarker.scale.y = 0.18;
    poseMarker.scale.z = 0.30;
    poseMarker.pose.orientation.w = 1.0;
    geometry_msgs::Point arrow[2];
    arrow[0].x = state.position[0]; arrow[0].y = state.position[1]; arrow[0].z = state.position[2];
    arrow[1].x = state.position[0] + 1.0*state.velocity[0]/state.velocity.norm(); 
    arrow[1].y = state.position[1] + 1.0*state.velocity[1]/state.velocity.norm(); 
    arrow[1].z = state.position[2] + 1.0*state.velocity[2]/state.velocity.norm();
    // std::cout << "vx:" << state.velocity[0] << std::endl;
    // std::cout << "vy:" << state.velocity[1] << std::endl;
    // std::cout << "vz:" << state.velocity[2] << std::endl;
    poseMarker.points.push_back(arrow[0]);
    poseMarker.points.push_back(arrow[1]);
    if (state.velocity.norm() > 0.01)
      poses.markers.push_back(poseMarker);
  }
  objectPosePub.publish(poses);
  edgePub.publish(boxes);

}

}
