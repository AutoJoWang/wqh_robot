#ifndef AXIOMA_MPC_CONTROLLER__MPC_CONTROLLER_PLUGIN_HPP_
#define AXIOMA_MPC_CONTROLLER__MPC_CONTROLLER_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav_msgs/msg/path.hpp"

#include "axioma_mpc_controller/mpc_controller.hpp"
#include "axioma_mpc_controller/path_handler.hpp"
#include "axioma_mpc_controller/robot_model.hpp"

namespace axioma_mpc_controller
{

/**
 * @brief MPC Controller Plugin for Nav2
 * 
 * 实现 Nav2 控制器接口，使用 MPC 进行路径跟踪
 */
class MPCControllerPlugin : public nav2_core::Controller
{
public:
  /**
   * @brief 构造函数
   */
  MPCControllerPlugin();

  /**
   * @brief 配置控制器
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief 清理资源
   */
  void cleanup() override;

  /**
   * @brief 激活控制器
   */
  void activate() override;

  /**
   * @brief 停用控制器
   */
  void deactivate() override;

  /**
   * @brief 设置路径
   */
  void setPlan(const nav_msgs::msg::Path & path) override;

  /**
   * @brief 计算速度命令
   */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief 设置速度限制
   */
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  /**
   * @brief 从参数服务器加载参数
   */
  void loadParameters();

  /**
   * @brief 获取机器人当前位姿
   */
  bool getRobotPose(
    geometry_msgs::msg::PoseStamped & global_pose,
    geometry_msgs::msg::PoseStamped & robot_pose);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("MPCControllerPlugin")};

  // MPC 控制器
  std::shared_ptr<MPCController> mpc_controller_;
  std::shared_ptr<PathHandler> path_handler_;
  
  // 参数
  double max_vel_x_;
  double min_vel_x_;
  double max_vel_theta_;
  double min_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  
  size_t prediction_horizon_;
  double dt_;
  std::vector<double> Q_;
  std::vector<double> R_;
  std::vector<double> P_;

  // costmap / obstacle handling params
  double footprint_radius_;
  int lethal_cost_threshold_;
  double obstacle_cost_weight_;
  double unknown_cost_penalty_;
  double stop_penalty_;
  double stop_vel_threshold_;
  double stop_omega_threshold_;
  int obstacle_sample_points_;
  
  bool initialized_;
  nav_msgs::msg::Path global_plan_;
};

}  // namespace axioma_mpc_controller

#endif  // AXIOMA_MPC_CONTROLLER__MPC_CONTROLLER_PLUGIN_HPP_
