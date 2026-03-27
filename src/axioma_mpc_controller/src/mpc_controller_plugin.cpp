#include "axioma_mpc_controller/mpc_controller_plugin.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>

namespace axioma_mpc_controller
{

MPCControllerPlugin::MPCControllerPlugin()
: costmap_(nullptr),
  max_vel_x_(0.26),
  min_vel_x_(0.0),
  max_vel_theta_(1.0),
  min_vel_theta_(0.0),
  acc_lim_x_(2.5),
  acc_lim_theta_(3.2),
  prediction_horizon_(10),
  dt_(0.1),
  footprint_radius_(0.22),
  lethal_cost_threshold_(253),
  obstacle_cost_weight_(200.0),
  unknown_cost_penalty_(10.0),
  stop_penalty_(50.0),
  stop_vel_threshold_(0.01),
  stop_omega_threshold_(0.05),
  obstacle_sample_points_(8),
  initialized_(false)
{
}

void MPCControllerPlugin::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();
  
  plugin_name_ = name;
  logger_ = node->get_logger();
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  
  RCLCPP_INFO(logger_, "Configuring MPC Controller Plugin");
  
  // 加载参数
  loadParameters();
  
  // 初始化路径处理器
  path_handler_ = std::make_shared<PathHandler>();
  
  // 初始化MPC控制器
  mpc_controller_ = std::make_shared<MPCController>();
  
  // 设置权重矩阵
  Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
  if (Q_.size() >= 3) {
    Q(0, 0) = Q_[0];
    Q(1, 1) = Q_[1];
    Q(2, 2) = Q_[2];
  }
  
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
  if (R_.size() >= 2) {
    R(0, 0) = R_[0];
    R(1, 1) = R_[1];
  }
  
  Eigen::Matrix3d P = Eigen::Matrix3d::Identity();
  if (P_.size() >= 3) {
    P(0, 0) = P_[0];
    P(1, 1) = P_[1];
    P(2, 2) = P_[2];
  }
  
  Eigen::Vector2d u_min(min_vel_x_, min_vel_theta_);
  Eigen::Vector2d u_max(max_vel_x_, max_vel_theta_);
  
  mpc_controller_->initialize(prediction_horizon_, dt_, Q, R, P, u_min, u_max);
  mpc_controller_->setPathHandler(path_handler_);
  mpc_controller_->setCostmap(costmap_);
  mpc_controller_->setFootprintRadius(footprint_radius_);
  mpc_controller_->setLethalCostThreshold(
    static_cast<unsigned char>(std::clamp(lethal_cost_threshold_, 0, 255)));
  mpc_controller_->setObstacleCostWeight(obstacle_cost_weight_);
  mpc_controller_->setUnknownCostPenalty(unknown_cost_penalty_);
  mpc_controller_->setStopPenalty(stop_penalty_);
  mpc_controller_->setStopVelocityThreshold(stop_vel_threshold_);
  mpc_controller_->setStopOmegaThreshold(stop_omega_threshold_);
  mpc_controller_->setObstacleSamplePoints(obstacle_sample_points_);
  
  initialized_ = true;
  RCLCPP_INFO(logger_, "MPC Controller Plugin configured");
}

void MPCControllerPlugin::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up MPC Controller Plugin");
  initialized_ = false;
}

void MPCControllerPlugin::activate()
{
  RCLCPP_INFO(logger_, "Activating MPC Controller Plugin");
}

void MPCControllerPlugin::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating MPC Controller Plugin");
}

void MPCControllerPlugin::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (path_handler_) {
    path_handler_->setPath(path);
  }
}

geometry_msgs::msg::TwistStamped MPCControllerPlugin::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & /* pose */,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /* goal_checker */)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_.lock()->now();
  cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
  
  if (!initialized_ || !path_handler_ || !path_handler_->isPathValid()) {
    RCLCPP_WARN(logger_, "MPC Controller not initialized or path invalid");
    return cmd_vel;
  }
  
  // 获取机器人当前位姿
  geometry_msgs::msg::PoseStamped global_pose;
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!getRobotPose(global_pose, robot_pose)) {
    RCLCPP_WARN(logger_, "Could not get robot pose");
    return cmd_vel;
  }
  
  // 转换为Eigen格式
  Eigen::Vector3d current_state;
  // Use pose in the controller's global frame (same frame as the transformed plan).
  current_state(0) = global_pose.pose.position.x;
  current_state(1) = global_pose.pose.position.y;
  
  tf2::Quaternion q(
    global_pose.pose.orientation.x,
    global_pose.pose.orientation.y,
    global_pose.pose.orientation.z,
    global_pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  current_state(2) = yaw;
  
  // 生成参考轨迹
  auto ref_trajectory = path_handler_->generateReferenceTrajectory(
    current_state, prediction_horizon_, dt_);
  
  // 计算MPC控制输入
  auto control = mpc_controller_->computeControl(current_state, ref_trajectory);
  
  // 应用速度限制
  double v = std::clamp(control(0), min_vel_x_, max_vel_x_);
  double omega = std::clamp(control(1), min_vel_theta_, max_vel_theta_);
  
  // 考虑加速度限制（简化处理）
  // 实际应该考虑当前速度
  double current_v = std::sqrt(velocity.linear.x * velocity.linear.x + 
                              velocity.linear.y * velocity.linear.y);
  double v_diff = v - current_v;
  if (std::abs(v_diff) > acc_lim_x_ * dt_) {
    v = current_v + std::copysign(acc_lim_x_ * dt_, v_diff);
  }
  
  double current_omega = velocity.angular.z;
  double omega_diff = omega - current_omega;
  if (std::abs(omega_diff) > acc_lim_theta_ * dt_) {
    omega = current_omega + std::copysign(acc_lim_theta_ * dt_, omega_diff);
  }
  
  // 设置速度命令
  cmd_vel.twist.linear.x = v;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.angular.z = omega;
  
  return cmd_vel;
}

void MPCControllerPlugin::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_vel_x_ = speed_limit * max_vel_x_;
  } else {
    max_vel_x_ = speed_limit;
  }
}

void MPCControllerPlugin::loadParameters()
{
  auto node = node_.lock();
  
  // 速度限制
  node->declare_parameter(plugin_name_ + ".max_vel_x", rclcpp::ParameterValue(0.26));
  node->declare_parameter(plugin_name_ + ".min_vel_x", rclcpp::ParameterValue(0.0));
  node->declare_parameter(plugin_name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
  node->declare_parameter(plugin_name_ + ".min_vel_theta", rclcpp::ParameterValue(0.0));
  
  // 加速度限制
  node->declare_parameter(plugin_name_ + ".acc_lim_x", rclcpp::ParameterValue(2.5));
  node->declare_parameter(plugin_name_ + ".acc_lim_theta", rclcpp::ParameterValue(3.2));
  
  // MPC 参数
  node->declare_parameter(plugin_name_ + ".prediction_horizon", rclcpp::ParameterValue(10));
  node->declare_parameter(plugin_name_ + ".dt", rclcpp::ParameterValue(0.1));
  node->declare_parameter(plugin_name_ + ".Q", rclcpp::ParameterValue(std::vector<double>{10.0, 10.0, 5.0}));
  node->declare_parameter(plugin_name_ + ".R", rclcpp::ParameterValue(std::vector<double>{1.0, 1.0}));
  node->declare_parameter(plugin_name_ + ".P", rclcpp::ParameterValue(std::vector<double>{50.0, 50.0, 20.0}));

  // costmap / obstacle params
  node->declare_parameter(plugin_name_ + ".footprint_radius", rclcpp::ParameterValue(0.22));
  node->declare_parameter(plugin_name_ + ".lethal_cost_threshold", rclcpp::ParameterValue(253));
  node->declare_parameter(plugin_name_ + ".obstacle_cost_weight", rclcpp::ParameterValue(200.0));
  node->declare_parameter(plugin_name_ + ".unknown_cost_penalty", rclcpp::ParameterValue(10.0));
  node->declare_parameter(plugin_name_ + ".stop_penalty", rclcpp::ParameterValue(50.0));
  node->declare_parameter(plugin_name_ + ".stop_vel_threshold", rclcpp::ParameterValue(0.01));
  node->declare_parameter(plugin_name_ + ".stop_omega_threshold", rclcpp::ParameterValue(0.05));
  node->declare_parameter(plugin_name_ + ".obstacle_sample_points", rclcpp::ParameterValue(8));
  
  // 获取参数
  node->get_parameter(plugin_name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(plugin_name_ + ".min_vel_x", min_vel_x_);
  node->get_parameter(plugin_name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(plugin_name_ + ".min_vel_theta", min_vel_theta_);
  node->get_parameter(plugin_name_ + ".acc_lim_x", acc_lim_x_);
  node->get_parameter(plugin_name_ + ".acc_lim_theta", acc_lim_theta_);
  int64_t prediction_horizon_param = 10;
  node->get_parameter(plugin_name_ + ".prediction_horizon", prediction_horizon_param);
  prediction_horizon_ = static_cast<size_t>(std::max<int64_t>(1, prediction_horizon_param));
  node->get_parameter(plugin_name_ + ".dt", dt_);
  node->get_parameter(plugin_name_ + ".Q", Q_);
  node->get_parameter(plugin_name_ + ".R", R_);
  node->get_parameter(plugin_name_ + ".P", P_);

  node->get_parameter(plugin_name_ + ".footprint_radius", footprint_radius_);
  node->get_parameter(plugin_name_ + ".lethal_cost_threshold", lethal_cost_threshold_);
  node->get_parameter(plugin_name_ + ".obstacle_cost_weight", obstacle_cost_weight_);
  node->get_parameter(plugin_name_ + ".unknown_cost_penalty", unknown_cost_penalty_);
  node->get_parameter(plugin_name_ + ".stop_penalty", stop_penalty_);
  node->get_parameter(plugin_name_ + ".stop_vel_threshold", stop_vel_threshold_);
  node->get_parameter(plugin_name_ + ".stop_omega_threshold", stop_omega_threshold_);
  node->get_parameter(plugin_name_ + ".obstacle_sample_points", obstacle_sample_points_);
}

bool MPCControllerPlugin::getRobotPose(
  geometry_msgs::msg::PoseStamped & global_pose,
  geometry_msgs::msg::PoseStamped & robot_pose)
{
  // Create an identity pose in base frame, then transform it into the controller global frame.
  robot_pose.header.frame_id = costmap_ros_->getBaseFrameID();
  robot_pose.header.stamp = rclcpp::Time();
  robot_pose.pose.position.x = 0.0;
  robot_pose.pose.position.y = 0.0;
  robot_pose.pose.position.z = 0.0;
  robot_pose.pose.orientation.x = 0.0;
  robot_pose.pose.orientation.y = 0.0;
  robot_pose.pose.orientation.z = 0.0;
  robot_pose.pose.orientation.w = 1.0;
  
  try {
    auto transform = tf_->lookupTransform(
      costmap_ros_->getGlobalFrameID(),
      costmap_ros_->getBaseFrameID(),
      tf2::TimePointZero);
    
    tf2::doTransform(robot_pose, global_pose, transform);
    global_pose.header.frame_id = costmap_ros_->getGlobalFrameID();
    global_pose.header.stamp = robot_pose.header.stamp;
    
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Could not get robot pose: %s", ex.what());
    return false;
  }
}

}  // namespace axioma_mpc_controller

// 注册插件
PLUGINLIB_EXPORT_CLASS(axioma_mpc_controller::MPCControllerPlugin, nav2_core::Controller)
