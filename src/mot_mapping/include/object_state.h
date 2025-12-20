// src/mot_mapping/include/object_state.h
#ifndef OBJECT_STATE_H
#define OBJECT_STATE_H

#include <eigen3/Eigen/Dense>

// 将 ObjectState 结构体定义放在这里
struct ObjectState {
  int id; 
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d size;
};

#endif