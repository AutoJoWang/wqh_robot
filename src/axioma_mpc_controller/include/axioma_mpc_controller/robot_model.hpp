#ifndef AXIOMA_MPC_CONTROLLER__ROBOT_MODEL_HPP_
#define AXIOMA_MPC_CONTROLLER__ROBOT_MODEL_HPP_

#include <Eigen/Dense>

namespace axioma_mpc_controller
{

/**
 * @brief 差速驱动机器人运动模型
 * 
 * 状态: x = [x, y, theta]^T
 * 控制: u = [v, omega]^T
 * 
 * 运动学模型:
 *   x_{k+1} = x_k + v_k * cos(theta_k) * dt
 *   y_{k+1} = y_k + v_k * sin(theta_k) * dt
 *   theta_{k+1} = theta_k + omega_k * dt
 */
class RobotModel
{
public:
  /**
   * @brief 构造函数
   * @param dt 时间步长（秒）
   */
  explicit RobotModel(double dt = 0.1);

  /**
   * @brief 设置时间步长
   */
  void setTimeStep(double dt);

  /**
   * @brief 前向运动学：根据当前状态和控制输入计算下一状态
   * @param state 当前状态 [x, y, theta]
   * @param control 控制输入 [v, omega]
   * @return 下一状态 [x, y, theta]
   */
  Eigen::Vector3d forwardKinematics(
    const Eigen::Vector3d & state,
    const Eigen::Vector2d & control) const;

  /**
   * @brief 计算雅可比矩阵（用于线性化）
   * @param state 当前状态
   * @param control 控制输入
   * @return A矩阵（状态雅可比）和 B矩阵（控制雅可比）
   */
  std::pair<Eigen::Matrix3d, Eigen::Matrix<double, 3, 2>> computeJacobians(
    const Eigen::Vector3d & state,
    const Eigen::Vector2d & control) const;

  /**
   * @brief 获取时间步长
   */
  double getTimeStep() const {return dt_;}

private:
  double dt_;  // 时间步长
};

}  // namespace axioma_mpc_controller

#endif  // AXIOMA_MPC_CONTROLLER__ROBOT_MODEL_HPP_
