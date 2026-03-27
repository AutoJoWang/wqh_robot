#ifndef AXIOMA_MPC_CONTROLLER__PATH_HANDLER_HPP_
#define AXIOMA_MPC_CONTROLLER__PATH_HANDLER_HPP_

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <vector>

namespace axioma_mpc_controller
{

/**
 * @brief 路径处理工具
 * 
 * 功能：
 * 1. 将全局路径转换为局部参考轨迹
 * 2. 找到路径上距离机器人最近的点
 * 3. 生成预测时域内的参考轨迹
 */
class PathHandler
{
public:
  /**
   * @brief 构造函数
   */
  PathHandler();

  /**
   * @brief 设置全局路径
   * @param path 全局路径（nav_msgs::Path）
   */
  void setPath(const nav_msgs::msg::Path & path);

  /**
   * @brief 找到路径上距离机器人最近的点
   * @param robot_pose 机器人当前位姿 [x, y, theta]
   * @return 最近点的索引
   */
  size_t findClosestPoint(const Eigen::Vector3d & robot_pose) const;

  /**
   * @brief 生成预测时域内的参考轨迹
   * @param robot_pose 机器人当前位姿
   * @param horizon 预测时域长度（步数）
   * @param dt 时间步长
   * @return 参考轨迹 [x_ref, y_ref, theta_ref] for each step
   */
  std::vector<Eigen::Vector3d> generateReferenceTrajectory(
    const Eigen::Vector3d & robot_pose,
    size_t horizon,
    double dt) const;

  /**
   * @brief 检查路径是否有效
   */
  bool isPathValid() const {return !path_poses_.empty();}

  /**
   * @brief 获取路径长度
   */
  size_t getPathLength() const {return path_poses_.size();}

private:
  std::vector<Eigen::Vector3d> path_poses_;  // 路径点 [x, y, theta]
  
  /**
   * @brief 从 PoseStamped 提取位姿
   */
  Eigen::Vector3d poseToState(const geometry_msgs::msg::PoseStamped & pose) const;
};

}  // namespace axioma_mpc_controller

#endif  // AXIOMA_MPC_CONTROLLER__PATH_HANDLER_HPP_
