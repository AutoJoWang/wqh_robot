#include "axioma_mpc_controller/mpc_controller.hpp"
#include <algorithm>
#include <limits>
#include <cmath>
#include <numeric>
#include "nav2_costmap_2d/cost_values.hpp"

namespace axioma_mpc_controller
{

MPCController::MPCController()
: horizon_(10),
  Q_(Eigen::Matrix3d::Identity()),
  R_(Eigen::Matrix2d::Identity()),
  P_(Eigen::Matrix3d::Identity()),
  u_min_(Eigen::Vector2d::Zero()),
  u_max_(Eigen::Vector2d::Zero())
{
  robot_model_ = std::make_shared<RobotModel>(0.1);
}

void MPCController::initialize(
  size_t horizon,
  double dt,
  const Eigen::Matrix3d & Q,
  const Eigen::Matrix2d & R,
  const Eigen::Matrix3d & P,
  const Eigen::Vector2d & u_min,
  const Eigen::Vector2d & u_max)
{
  horizon_ = horizon;
  Q_ = Q;
  R_ = R;
  P_ = P;
  u_min_ = u_min;
  u_max_ = u_max;
  
  robot_model_->setTimeStep(dt);
}

Eigen::Vector2d MPCController::computeControl(
  const Eigen::Vector3d & current_state,
  const std::vector<Eigen::Vector3d> & ref_trajectory)
{
  return solveMPC(current_state, ref_trajectory);
}

void MPCController::setPathHandler(std::shared_ptr<PathHandler> path_handler)
{
  path_handler_ = path_handler;
}

double MPCController::obstacleCostAt(double x, double y) const
{
  if (!costmap_) {
    return 0.0;
  }

  // Sample a circular footprint approximation (center + ring points).
  // Returns +inf if any sample is outside the map or in lethal cost.
  const int n = std::max(0, obstacle_sample_points_);
  const double r = std::max(0.0, footprint_radius_);
  const double two_pi = 2.0 * M_PI;

  auto sample_cell_cost = [&](double wx, double wy) -> double {
    unsigned int mx = 0;
    unsigned int my = 0;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      return std::numeric_limits<double>::infinity();
    }
    const unsigned char c = costmap_->getCost(mx, my);
    if (c == nav2_costmap_2d::NO_INFORMATION) {
      return unknown_cost_penalty_;
    }
    if (c >= lethal_cost_threshold_) {
      return std::numeric_limits<double>::infinity();
    }
    // Scale 0..252 into 0..1, then square for sharper repulsion near obstacles.
    const double s = static_cast<double>(c) / 252.0;
    return obstacle_cost_weight_ * s * s;
  };

  double total = sample_cell_cost(x, y);
  if (!std::isfinite(total)) {
    return total;
  }

  for (int i = 0; i < n; ++i) {
    const double a = two_pi * static_cast<double>(i) / static_cast<double>(n);
    const double wx = x + r * std::cos(a);
    const double wy = y + r * std::sin(a);
    const double c = sample_cell_cost(wx, wy);
    if (!std::isfinite(c)) {
      return c;
    }
    total += c;
  }

  return total;
}

double MPCController::computeCost(
  const Eigen::Vector3d & state,
  const Eigen::Vector2d & control,
  const Eigen::Vector3d & ref_state,
  bool is_terminal) const
{
  Eigen::Vector3d state_error = state - ref_state;
  
  // 角度误差归一化到 [-pi, pi]
  while (state_error(2) > M_PI) {
    state_error(2) -= 2.0 * M_PI;
  }
  while (state_error(2) < -M_PI) {
    state_error(2) += 2.0 * M_PI;
  }
  
  double state_cost = state_error.transpose() * (is_terminal ? P_ : Q_) * state_error;
  double control_cost = control.transpose() * R_ * control;
  
  return state_cost + control_cost;
}

Eigen::Vector2d MPCController::solveMPC(
  const Eigen::Vector3d & current_state,
  const std::vector<Eigen::Vector3d> & ref_trajectory)
{
  // 简化实现：使用滚动时域优化
  // 注意：这是一个简化版本，实际应使用CasADi+IPOPT进行非线性优化
  
  predicted_trajectory_.clear();
  predicted_trajectory_.push_back(current_state);
  
  // 如果参考轨迹为空，返回零控制
  if (ref_trajectory.empty()) {
    return Eigen::Vector2d::Zero();
  }
  
  // 简化策略：使用贪心方法选择第一个控制输入
  // 实际MPC应该优化整个时域的控制序列
  
  double best_cost = std::numeric_limits<double>::max();
  Eigen::Vector2d best_control = Eigen::Vector2d::Zero();

  const double dist_to_ref0 =
    (current_state.head<2>() - ref_trajectory.front().head<2>()).norm();
  
  // 采样控制空间（类似DWB，但用于MPC的第一步）
  const int v_samples = 10;
  const int omega_samples = 10;
  
  for (int i = 0; i <= v_samples; ++i) {
    for (int j = 0; j <= omega_samples; ++j) {
      double v = u_min_(0) + (u_max_(0) - u_min_(0)) * i / v_samples;
      double omega = u_min_(1) + (u_max_(1) - u_min_(1)) * j / omega_samples;
      
      Eigen::Vector2d control(v, omega);
      
      // 预测未来轨迹并计算总代价
      double total_cost = 0.0;
      Eigen::Vector3d state = current_state;

      // If we're not close to the reference, discourage selecting a near-zero command.
      if (dist_to_ref0 > 0.20 &&
        std::abs(v) < stop_vel_threshold_ &&
        std::abs(omega) < stop_omega_threshold_)
      {
        total_cost += stop_penalty_;
      }
      
      for (size_t k = 0; k < std::min(horizon_, ref_trajectory.size()); ++k) {
        // 预测下一状态
        state = robot_model_->forwardKinematics(state, control);
        
        const double obs_cost = obstacleCostAt(state(0), state(1));
        if (!std::isfinite(obs_cost)) {
          total_cost = std::numeric_limits<double>::infinity();
          break;
        }
        total_cost += obs_cost;

        // 计算代价
        bool is_terminal = (k == horizon_ - 1 || k == ref_trajectory.size() - 1);
        total_cost += computeCost(state, control, ref_trajectory[k], is_terminal);
      }
      
      if (total_cost < best_cost) {
        best_cost = total_cost;
        best_control = control;
      }
    }
  }
  
  // 生成预测轨迹用于可视化
  Eigen::Vector3d state = current_state;
  for (size_t k = 0; k < std::min(horizon_, ref_trajectory.size()); ++k) {
    state = robot_model_->forwardKinematics(state, best_control);
    predicted_trajectory_.push_back(state);
  }
  
  return best_control;
}

}  // namespace axioma_mpc_controller
