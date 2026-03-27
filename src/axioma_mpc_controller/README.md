# Axioma MPC Controller

MPC (Model Predictive Control) 路径跟踪控制器，用于替换 Nav2 的 DWB 控制器。

## 功能特性

- ✅ 模型预测控制算法
- ✅ 差速驱动机器人运动模型
- ✅ 路径跟踪与避障
- ✅ Nav2 插件接口
- ✅ 可配置的预测时域和权重矩阵

## 依赖

### ROS 2 依赖
- `nav2_core`
- `nav2_util`
- `nav2_costmap_2d`
- `geometry_msgs`
- `nav_msgs`
- `tf2_ros`
- `rclcpp`
- `rclcpp_lifecycle`
- `pluginlib`

### 系统依赖
- `libeigen3-dev` - Eigen3 矩阵库
- `eigen3_cmake_module` - Eigen3 CMake 模块

安装依赖：
```bash
sudo apt install libeigen3-dev ros-humble-eigen3-cmake-module
```

## 编译

```bash
cd /home/s105/ws/wqh_ws/wqh_robot
colcon build --packages-select axioma_mpc_controller
source install/setup.bash
```

## 使用方法

### 1. 在 nav2_params.yaml 中配置

Nav2 自带行为树里 `FollowPath` 节点的 `controller_id` 固定为 **`FollowPath`**。必须把 MPC 注册为同名插件（只改 `plugin` 类型即可），**不要**改成 `MPCPath`，否则行为树仍请求 `FollowPath`，控制器对不上，小车不会动。

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "axioma_mpc_controller::MPCControllerPlugin"
      max_vel_x: 0.26
      min_vel_theta: -1.0   # 与 max_vel_theta 对称，差速转向需要负角速度
      max_vel_theta: 1.0
      prediction_horizon: 10
      dt: 0.1
      Q: [10.0, 10.0, 5.0]
      R: [1.0, 1.0]
      P: [50.0, 50.0, 20.0]
```

### 2. 启动导航

```bash
ros2 launch axioma_bringup navigation_bringup.launch.py
```

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_vel_x` | double | 0.26 | 最大线速度 (m/s) |
| `min_vel_x` | double | 0.0 | 最小线速度 (m/s) |
| `max_vel_theta` | double | 1.0 | 最大角速度 (rad/s) |
| `min_vel_theta` | double | -1.0 | 最小角速度 (rad/s)，差速车建议取负的 max |
| `acc_lim_x` | double | 2.5 | 线加速度限制 (m/s²) |
| `acc_lim_theta` | double | 3.2 | 角加速度限制 (rad/s²) |
| `prediction_horizon` | int | 10 | 预测时域长度（步数） |
| `dt` | double | 0.1 | 时间步长（秒） |
| `Q` | double[3] | [10.0, 10.0, 5.0] | 状态权重 [x, y, theta] |
| `R` | double[2] | [1.0, 1.0] | 控制权重 [v, omega] |
| `P` | double[3] | [50.0, 50.0, 20.0] | 终端权重 [x, y, theta] |

## 算法说明

### MPC 优化问题

```
minimize:  J = Σ_{k=0}^{N-1} [||x_k - x_ref_k||²_Q + ||u_k||²_R] + ||x_N - x_ref_N||²_P
subject to:
    x_{k+1} = f(x_k, u_k)           (运动模型约束)
    u_min ≤ u_k ≤ u_max             (控制约束)
```

其中：
- `x_k = [x, y, theta]^T` - 状态（位置和朝向）
- `u_k = [v, omega]^T` - 控制输入（线速度和角速度）
- `N` - 预测时域长度
- `Q, R, P` - 权重矩阵

### 运动模型

差速驱动机器人运动学模型（欧拉离散化）：

```
x_{k+1} = x_k + v_k * cos(theta_k) * dt
y_{k+1} = y_k + v_k * sin(theta_k) * dt
theta_{k+1} = theta_k + omega_k * dt
```

## 当前实现状态

### ✅ 已完成
- Nav2 控制器插件接口
- 差速驱动运动模型
- 路径处理工具
- MPC 控制器框架
- 参数配置

### ⚠️ 当前限制
- **简化优化求解器**：当前使用采样方法（类似DWB），不是真正的非线性优化
- **未集成 CasADi**：未来可以集成 CasADi+IPOPT 进行真正的非线性优化
- **障碍物约束**：当前未在MPC优化中显式考虑障碍物（依赖Nav2的代价地图）

### 🔄 未来改进
1. 集成 CasADi + IPOPT 进行真正的非线性优化
2. 在MPC约束中显式添加障碍物避让
3. 支持滑移率估计与补偿
4. 性能优化（减少计算时间）

## 与 DWB 对比

| 特性 | DWB | MPC (当前) | MPC (未来) |
|------|-----|-----------|-----------|
| 优化方法 | 采样+评价 | 采样（简化） | 非线性优化 |
| 预测时域 | 1.7秒 | 可配置 | 可配置 |
| 计算时间 | < 5ms | 10-50ms | 20-100ms |
| 路径精度 | 中等 | 中等 | 高 |
| 约束处理 | 隐式 | 部分显式 | 完全显式 |

## 故障排除

### 编译错误：找不到 Eigen3
```bash
sudo apt install libeigen3-dev ros-humble-eigen3-cmake-module
```

### 运行时错误：插件未找到
确保已 source 工作空间：
```bash
source install/setup.bash
```

### 控制器不工作
检查：
1. `controller_plugins` 是否为 `["FollowPath"]`，且 `FollowPath.plugin` 为 MPC（与行为树 `controller_id` 一致）
2. 插件是否正确注册（检查 `mpc_controller_plugin.xml`），工作空间已 `source install/setup.bash`
3. 日志是否出现 `MPC Controller not initialized or path invalid`；`ros2 topic echo /cmd_vel` 是否非零

## 许可证

Apache-2.0
