#include "axioma_mpc_controller/robot_model.hpp"
#include <cmath>

namespace axioma_mpc_controller
{

RobotModel::RobotModel(double dt)
: dt_(dt)
{
}

void RobotModel::setTimeStep(double dt)
{
  dt_ = dt;
}

Eigen::Vector3d RobotModel::forwardKinematics(
  const Eigen::Vector3d & state,
  const Eigen::Vector2d & control) const
{
  const double x = state(0);
  const double y = state(1);
  const double theta = state(2);
  const double v = control(0);
  const double omega = control(1);

  Eigen::Vector3d next_state;
  
  // 差速驱动运动学模型（欧拉离散化）
  next_state(0) = x + v * std::cos(theta) * dt_;
  next_state(1) = y + v * std::sin(theta) * dt_;
  next_state(2) = theta + omega * dt_;

  // 归一化角度到 [-pi, pi]
  while (next_state(2) > M_PI) {
    next_state(2) -= 2.0 * M_PI;
  }
  while (next_state(2) < -M_PI) {
    next_state(2) += 2.0 * M_PI;
  }

  return next_state;
}

std::pair<Eigen::Matrix3d, Eigen::Matrix<double, 3, 2>> RobotModel::computeJacobians(
  const Eigen::Vector3d & state,
  const Eigen::Vector2d & control) const
{
  const double theta = state(2);
  const double v = control(0);

  // 状态雅可比矩阵 A = df/dx
  Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
  A(0, 2) = -v * std::sin(theta) * dt_;
  A(1, 2) = v * std::cos(theta) * dt_;

  // 控制雅可比矩阵 B = df/du
  Eigen::Matrix<double, 3, 2> B;
  B(0, 0) = std::cos(theta) * dt_;  // df/dv
  B(0, 1) = 0.0;
  B(1, 0) = std::sin(theta) * dt_;
  B(1, 1) = 0.0;
  B(2, 0) = 0.0;
  B(2, 1) = dt_;  // df/domega

  return std::make_pair(A, B);
}

}  // namespace axioma_mpc_controller
