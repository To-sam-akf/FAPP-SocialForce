// 修改 social_force_predictor.cpp
#include "social_force_predictor.h"
#include <cmath>
#include <iostream>

// 参数调整区 (Tune these!)
const double A_STRENGTH = 1.0;     // 排斥力强度 (越大避让越早)
const double B_RANGE = 0.5;        // 排斥力作用范围 (越大开始避让的距离越远)
const double VIEW_ANGLE = 200.0;   // 视野角度 (度)
const double DT = 0.2;             // 预测的时间步长 (秒)
const double LAMBDA = 0.1;         // 身后障碍物的权重 (0~1)

ForceComponents SocialForcePredictor::getForceComponents(const ObjectState& target, 
                                                        const std::vector<ObjectState>& neighbors) {
    ForceComponents components;
    
    // 1. 驱动力 (Driving Force)
    // 假设行人想要保持当前的目标速度方向和大小
    // F_drive = (Desired_Vel - Current_Vel) / relaxation_time
    // 这里简化：假设行人原本想保持当前速度，所以驱动力主要用于抵抗阻力
    // 更好的做法是：如果知道目标点，指向目标点；如果不知道，驱动力设为0，只靠惯性
    components.driving_force = Eigen::Vector3d(0, 0, 0); 
    
    components.repulsive_force = Eigen::Vector3d(0, 0, 0);
    
    for (int i = 0; i < neighbors.size(); ++i) {
        const auto& neighbor = neighbors[i];
        Eigen::Vector3d vec_to_neighbor = neighbor.position - target.position; // r_ij
        double distance = vec_to_neighbor.norm();
        
        // 避免除零和自身计算
        if (distance < 0.05) continue; 

        // --- 核心改进 A: 指数型排斥力 ---
        // 这种力随距离减小而指数级增加，比反比平方律更平滑且符合生物本能
        Eigen::Vector3d n_ij = vec_to_neighbor.normalized(); // 指向邻居的方向
        double repulsion_magnitude = A_STRENGTH * std::exp((0.3 - distance) / B_RANGE); 
        // 0.3 是假设的人体半径之和

        // --- 核心改进 B: 各向异性 (视野因子) ---
        // 计算邻居是否在行人前方
        Eigen::Vector3d vel_dir = target.velocity.normalized();
        double cos_phi = -n_ij.dot(vel_dir); // 负号是因为排斥力方向是背离邻居的
        
        double w = LAMBDA; // 默认权重（身后）
        // 如果邻居在视线范围内 (cos_phi > cos(100度))
        if (cos_phi >= std::cos(VIEW_ANGLE / 2.0 * M_PI / 180.0)) {
            w = 1.0; // 前方权重
        }
        
        // 排斥力方向是 邻居->我 (即 -vec_to_neighbor)
        Eigen::Vector3d individual_repulsion = -n_ij * repulsion_magnitude * w;
        
        components.individual_repulsions.push_back(std::make_pair(individual_repulsion, i));
        components.repulsive_force += individual_repulsion;
    }
    
    // 3. 合成力
    components.total_force = components.driving_force + components.repulsive_force;
    
    return components;
}

Eigen::Vector3d SocialForcePredictor::predictTrajectory(const ObjectState& target, 
                                                        const std::vector<ObjectState>& neighbors) {
    // 1. 先计算基础的社会力 (获取 total_force)
    ForceComponents components = getForceComponents(target, neighbors);
    
    // ==================== 补全变量定义 ====================
    
    // [来源 1] force: 从刚刚计算的 components 中提取总力
    // 注意：这里不能用引用，因为我们需要在下面修改这个 force 的值
    Eigen::Vector3d force = components.total_force; 
    
    // [来源 2] speed: 从输入的 target 对象中获取当前速度的模长
    double speed = target.velocity.norm();
    
    // ====================================================

    // --- 修复核心：高速时的惯性约束 (方案一) ---
    if (speed > 0.1) { // 只有在移动时才处理，避免除零错误
        Eigen::Vector3d vel_dir = target.velocity.normalized();
        
        // 分解力：
        // force_parallel: 沿着速度方向的分量 (加速或减速)
        // force_perpendicular: 垂直于速度方向的分量 (转弯)
        Eigen::Vector3d force_parallel = force.dot(vel_dir) * vel_dir;
        Eigen::Vector3d force_perpendicular = force - force_parallel;

        // 动态权重计算：
        // 速度越快，speed 越大，分母越大，turning_factor 越小
        // 这意味着高速时，侧向力(转弯力)会被大幅削弱，模拟了“惯性”
        double turning_factor = 1.0 / (1.0 + 2.0 * speed); 
        
        // 重新合成力：保留全部的加减速力，但削弱转弯力
        force = force_parallel + force_perpendicular * turning_factor;
    }

    // --- 速度更新 (动力学积分) ---
    // 注意：这里使用的是修正后的 force，而不是原始的 components.total_force
    Eigen::Vector3d predicted_vel = target.velocity + force * DT;
    
    // 限制最大速度 (防止数值爆炸)
    double max_speed = 2.0; // 假设行人最大速度 2m/s
    if (predicted_vel.norm() > max_speed) {
        predicted_vel.normalize();
        predicted_vel *= max_speed;
    }
    
    return predicted_vel;
}

