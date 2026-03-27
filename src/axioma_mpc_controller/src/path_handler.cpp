#include "axioma_mpc_controller/path_handler.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>
#include <limits>

namespace axioma_mpc_controller
{

PathHandler::PathHandler()
{
}

void PathHandler::setPath(const nav_msgs::msg::Path & path)
{
  path_poses_.clear();
  
  for (const auto & pose_stamped : path.poses) {
    path_poses_.push_back(poseToState(pose_stamped));
  }
}

size_t PathHandler::findClosestPoint(const Eigen::Vector3d & robot_pose) const
{
  if (path_poses_.empty()) {
    return 0;
  }

  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < path_poses_.size(); ++i) {
    const auto & path_point = path_poses_[i];
    double dx = path_point(0) - robot_pose(0);
    double dy = path_point(1) - robot_pose(1);
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx;
}

std::vector<Eigen::Vector3d> PathHandler::generateReferenceTrajectory(
  const Eigen::Vector3d & robot_pose,
  size_t horizon,
  double /* dt */) const
{
  std::vector<Eigen::Vector3d> ref_traj;
  
  if (path_poses_.empty()) {
    // 如果没有路径，返回当前位置
    for (size_t i = 0; i < horizon; ++i) {
      ref_traj.push_back(robot_pose);
    }
    return ref_traj;
  }

  // 找到最近点
  size_t start_idx = findClosestPoint(robot_pose);
  
  // 生成参考轨迹
  for (size_t i = 0; i < horizon; ++i) {
    size_t idx = std::min(start_idx + i, path_poses_.size() - 1);
    ref_traj.push_back(path_poses_[idx]);
  }

  return ref_traj;
}

Eigen::Vector3d PathHandler::poseToState(const geometry_msgs::msg::PoseStamped & pose) const
{
  Eigen::Vector3d state;
  
  // 提取位置
  state(0) = pose.pose.position.x;
  state(1) = pose.pose.position.y;
  
  // 提取朝向（从四元数转换为欧拉角）
  tf2::Quaternion q(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w);
  
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  state(2) = yaw;
  
  return state;
}

}  // namespace axioma_mpc_controller
