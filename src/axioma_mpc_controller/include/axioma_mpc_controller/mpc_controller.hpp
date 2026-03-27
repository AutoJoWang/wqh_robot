#ifndef AXIOMA_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
#define AXIOMA_MPC_CONTROLLER__MPC_CONTROLLER_HPP_

#include "axioma_mpc_controller/robot_model.hpp"
#include "axioma_mpc_controller/path_handler.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace axioma_mpc_controller
{

/**
 * @brief MPC 控制器
 * 
 * 实现模型预测控制算法用于路径跟踪
 * 
 * 优化问题：
 *   minimize: J = Σ [||x_k - x_ref_k||²_Q + ||u_k||²_R] + ||x_N - x_ref_N||²_P
 *   subject to:
 *     x_{k+1} = f(x_k, u_k)  (运动模型)
 *     u_min ≤ u_k ≤ u_max    (控制约束)
 */
class MPCController
{
public:
  /**
   * @brief 构造函数
   */
  MPCController();

  /**
   * @brief 初始化控制器
   * @param horizon 预测时域长度
   * @param dt 时间步长
   * @param Q 状态权重矩阵 (3x3)
   * @param R 控制权重矩阵 (2x2)
   * @param P 终端权重矩阵 (3x3)
   * @param u_min 控制下界 [v_min, omega_min]
   * @param u_max 控制上界 [v_max, omega_max]
   */
  void initialize(
    size_t horizon,
    double dt,
    const Eigen::Matrix3d & Q,
    const Eigen::Matrix2d & R,
    const Eigen::Matrix3d & P,
    const Eigen::Vector2d & u_min,
    const Eigen::Vector2d & u_max);

  /**
   * @brief 计算控制输入
   * @param current_state 当前状态 [x, y, theta]
   * @param ref_trajectory 参考轨迹 [x_ref, y_ref, theta_ref] for each step
   * @return 最优控制输入 [v, omega]
   */
  Eigen::Vector2d computeControl(
    const Eigen::Vector3d & current_state,
    const std::vector<Eigen::Vector3d> & ref_trajectory);

  /**
   * @brief 设置路径处理器
   */
  void setPathHandler(std::shared_ptr<PathHandler> path_handler);

  void setCostmap(nav2_costmap_2d::Costmap2D * costmap) {costmap_ = costmap;}
  void setFootprintRadius(double radius) {footprint_radius_ = radius;}
  void setLethalCostThreshold(unsigned char threshold) {lethal_cost_threshold_ = threshold;}
  void setObstacleCostWeight(double weight) {obstacle_cost_weight_ = weight;}
  void setUnknownCostPenalty(double penalty) {unknown_cost_penalty_ = penalty;}
  void setStopPenalty(double penalty) {stop_penalty_ = penalty;}
  void setStopVelocityThreshold(double v_thresh) {stop_vel_threshold_ = v_thresh;}
  void setStopOmegaThreshold(double w_thresh) {stop_omega_threshold_ = w_thresh;}
  void setObstacleSamplePoints(int points) {obstacle_sample_points_ = points;}

  /**
   * @brief 获取预测轨迹（用于可视化）
   */
  std::vector<Eigen::Vector3d> getPredictedTrajectory() const
  {
    return predicted_trajectory_;
  }

private:
  double obstacleCostAt(double x, double y) const;

  /**
   * @brief 计算代价函数值
   */
  double computeCost(
    const Eigen::Vector3d & state,
    const Eigen::Vector2d & control,
    const Eigen::Vector3d & ref_state,
    bool is_terminal = false) const;

  /**
   * @brief 使用梯度下降法求解MPC（简化版本）
   * 注意：这是一个简化实现，实际应用中应使用CasADi+IPOPT
   */
  Eigen::Vector2d solveMPC(
    const Eigen::Vector3d & current_state,
    const std::vector<Eigen::Vector3d> & ref_trajectory);

  std::shared_ptr<RobotModel> robot_model_;
  std::shared_ptr<PathHandler> path_handler_;
  
  // MPC 参数
  size_t horizon_;
  Eigen::Matrix3d Q_;  // 状态权重
  Eigen::Matrix2d R_;   // 控制权重
  Eigen::Matrix3d P_;   // 终端权重
  Eigen::Vector2d u_min_;
  Eigen::Vector2d u_max_;

  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  unsigned char lethal_cost_threshold_{253};
  double obstacle_cost_weight_{200.0};
  double unknown_cost_penalty_{10.0};
  double footprint_radius_{0.22};
  double stop_penalty_{50.0};
  double stop_vel_threshold_{0.01};
  double stop_omega_threshold_{0.05};
  int obstacle_sample_points_{8};
  
  // 预测轨迹（用于可视化）
  std::vector<Eigen::Vector3d> predicted_trajectory_;
};

}  // namespace axioma_mpc_controller

#endif  // AXIOMA_MPC_CONTROLLER__MPC_CONTROLLER_HPP_
