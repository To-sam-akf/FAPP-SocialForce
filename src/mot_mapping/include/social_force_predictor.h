// 修改 social_force_predictor.h
#ifndef SOCIAL_FORCE_PREDICTOR_H
#define SOCIAL_FORCE_PREDICTOR_H
 
#include <vector>
#include <eigen3/Eigen/Dense>
 #include "object_state.h" // <--- 新增：包含公共头文件
// struct ObjectState;
 
struct ForceComponents {
    Eigen::Vector3d driving_force;
    Eigen::Vector3d repulsive_force;
    Eigen::Vector3d total_force;
    std::vector<std::pair<Eigen::Vector3d, int>> individual_repulsions; // 每个邻居的排斥力及其ID
};
 
class SocialForcePredictor {
public:
    Eigen::Vector3d predictTrajectory(const ObjectState& target, 
                                     const std::vector<ObjectState>& neighbors);
    
    // 新增：获取详细的力分析
    ForceComponents getForceComponents(const ObjectState& target, 
                                     const std::vector<ObjectState>& neighbors);
};
 
#endif