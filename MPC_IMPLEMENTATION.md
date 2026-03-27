# MPC 路径跟踪实现方案

## 当前路径跟踪算法分析

### 1. 当前使用的算法：**DWB (Dynamic Window Approach)**

#### 算法位置
- **配置文件**：`src/axioma_navigation/config/nav2_params.yaml`
- **插件名称**：`dwb_core::DWBLocalPlanner`
- **配置节**：`controller_server.FollowPath` (lines 173-233)

#### DWB 算法工作原理

```
输入：
├─ 全局路径 (/plan) - nav_msgs/Path
├─ 机器人当前位姿 (TF: base_link)
├─ 局部代价地图 (local_costmap)
└─ 当前速度 (/odom)

处理流程：
1. 速度空间采样
   ├─ vx_samples: 20 (x方向速度采样数)
   ├─ vy_samples: 5  (y方向速度采样数)
   └─ vtheta_samples: 20 (角速度采样数)
   
2. 轨迹生成与仿真
   ├─ sim_time: 1.7s (预测未来1.7秒的轨迹)
   ├─ linear_granularity: 0.05m (轨迹点间距)
   └─ angular_granularity: 0.025rad (角度间距)
   
3. 轨迹评价（使用7个评价函数）
   ├─ BaseObstacle: 障碍物避让（权重: 1.0）
   ├─ PathAlign: 路径对齐（权重: 32.0）
   ├─ GoalAlign: 目标对齐（权重: 24.0）
   ├─ PathDist: 路径距离（权重: 32.0）
   ├─ GoalDist: 目标距离（权重: 24.0）
   ├─ RotateToGoal: 旋转到目标（权重: 32.0）
   └─ Oscillation: 振荡检测

4. 选择最优轨迹
   └─ 输出速度命令 (/cmd_vel)

输出：
└─ geometry_msgs/Twist (线速度和角速度)
```

#### DWB 算法特点

**优点：**
- ✅ 实时性好（20 Hz）
- ✅ 避障能力强
- ✅ 参数可调，适应性强
- ✅ 已集成在 Nav2 中，稳定可靠

**缺点：**
- ❌ 基于采样，不是最优解
- ❌ 无法考虑长期预测（只预测1.7秒）
- ❌ 无法显式处理约束（如滑移率）
- ❌ 评价函数权重需要手动调节

---

## MPC 路径跟踪实现方案

### 1. MPC vs DWB 对比

| 特性 | DWB (当前) | MPC (目标) |
|------|-----------|-----------|
| **优化方法** | 采样 + 评价函数 | 数值优化（QP/NLP） |
| **预测时域** | 1.7秒（固定） | 可配置（通常3-10秒） |
| **约束处理** | 隐式（通过评价函数） | 显式（硬约束） |
| **滑移率** | 无法考虑 | 可以显式建模 |
| **计算复杂度** | 低（采样） | 中高（优化求解） |
| **实时性** | 高（20 Hz） | 中（10-20 Hz，取决于求解器） |
| **最优性** | 局部最优 | 全局最优（在预测时域内） |

### 2. 实现架构

```
┌─────────────────────────────────────────────────────────┐
│              Nav2 Controller Server                      │
│  (controller_server, 20 Hz)                              │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
         ┌───────────────────────┐
         │  Controller Plugin    │
         │  (nav2_core::Controller)│
         └───────────┬─────────────┘
                     │
         ┌───────────▼─────────────┐
         │   MPC Controller        │
         │   (新创建的插件)          │
         └───────────┬─────────────┘
                     │
    ┌────────────────┼────────────────┐
    │                │                │
    ▼                ▼                ▼
┌─────────┐   ┌──────────┐   ┌─────────────┐
│ CasADi  │   │ 机器人    │   │ 路径处理     │
│ 优化器   │   │ 运动模型   │   │ (路径转换)   │
└─────────┘   └──────────┘   └─────────────┘
```

### 3. 需要创建的组件

#### 3.1 新的 ROS 2 包：`axioma_mpc_controller`

```
axioma_mpc_controller/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── axioma_mpc_controller/
│       ├── mpc_controller.hpp      # MPC控制器主类
│       ├── robot_model.hpp         # 差速驱动机器人模型
│       └── path_handler.hpp        # 路径处理工具
├── src/
│   ├── mpc_controller.cpp          # MPC控制器实现
│   ├── robot_model.cpp             # 运动模型实现
│   ├── path_handler.cpp            # 路径处理实现
│   └── mpc_controller_plugin.cpp   # Nav2插件接口
└── config/
    └── mpc_params.yaml             # MPC参数配置
```

#### 3.2 核心依赖

```xml
<!-- package.xml -->
<depend>nav2_core</depend>           <!-- Nav2控制器接口 -->
<depend>nav2_util</depend>           <!-- Nav2工具类 -->
<depend>nav2_costmap_2d</depend>     <!-- 代价地图 -->
<depend>geometry_msgs</depend>        <!-- 位姿、速度消息 -->
<depend>nav_msgs</depend>             <!-- 路径消息 -->
<depend>tf2_ros</depend>              <!-- TF变换 -->
<depend>casadi</depend>               <!-- MPC优化库 -->
<depend>Eigen3</depend>               <!-- 矩阵运算 -->
```

### 4. MPC 控制器设计

#### 4.1 机器人运动模型（差速驱动）

```cpp
// 状态：x = [x, y, θ]^T
// 控制：u = [v, ω]^T
// 离散化模型（欧拉法）：
// x_{k+1} = x_k + v_k * cos(θ_k) * dt
// y_{k+1} = y_k + v_k * sin(θ_k) * dt
// θ_{k+1} = θ_k + ω_k * dt
```

#### 4.2 MPC 优化问题

```
minimize:  J = Σ_{k=0}^{N-1} [||x_k - x_ref_k||²_Q + ||u_k||²_R] + ||x_N - x_ref_N||²_P
subject to:
    x_{k+1} = f(x_k, u_k)           (运动模型约束)
    x_min ≤ x_k ≤ x_max             (状态约束)
    u_min ≤ u_k ≤ u_max             (控制约束)
    |v_k| ≤ v_max                   (速度约束)
    |ω_k| ≤ ω_max                   (角速度约束)
    costmap(x_k, y_k) < threshold   (障碍物约束)
```

其中：
- `N`: 预测时域长度（horizon）
- `Q`: 状态权重矩阵（路径跟踪误差）
- `R`: 控制权重矩阵（平滑性）
- `P`: 终端权重矩阵（终端误差）

#### 4.3 滑移率扩展（可选，未来实现）

```cpp
// 扩展状态：x = [x, y, θ, s_l, s_r]^T
// s_l, s_r: 左右轮滑移率
// 控制：u = [v_l, v_r]^T (左右轮速度)
// 
// 运动模型（考虑滑移）：
// v = (v_l * (1 - s_l) + v_r * (1 - s_r)) / 2
// ω = (v_r * (1 - s_r) - v_l * (1 - s_l)) / wheelbase
```

### 5. 实现步骤

#### 阶段 1：基础 MPC 控制器（无滑移率）

1. **创建 ROS 2 包**
   ```bash
   cd src
   ros2 pkg create --build-type ament_cmake axioma_mpc_controller
   ```

2. **实现 Nav2 控制器接口**
   - 继承 `nav2_core::Controller`
   - 实现必要的方法：
     - `configure()`: 初始化
     - `setPlan()`: 设置全局路径
     - `computeVelocityCommands()`: 计算速度命令
     - `isGoalReached()`: 检查是否到达目标

3. **集成 CasADi**
   - 安装 CasADi: `sudo apt install libcasadi-dev`
   - 在 CMakeLists.txt 中链接
   - 实现 MPC 优化问题

4. **测试与调试**
   - 单元测试（运动模型）
   - 仿真测试（Gazebo）
   - 参数调节

#### 阶段 2：添加障碍物约束

1. **代价地图集成**
   - 从 `local_costmap` 获取障碍物信息
   - 在 MPC 约束中添加障碍物避让

2. **优化求解器选择**
   - IPOPT（非线性优化，更精确）
   - qpOASES（二次规划，更快）

#### 阶段 3：滑移率估计与补偿（未来）

1. **EKF 滑移率估计**
   - 创建 `axioma_ekf_slip` 包
   - 使用 IMU + 里程计估计滑移率
   - 发布滑移率话题

2. **MPC 滑移率补偿**
   - 在运动模型中考虑滑移率
   - 使用 EKF 估计值作为状态

### 6. 配置文件示例

```yaml
# nav2_params.yaml（插件名须为 FollowPath，与默认行为树 controller_id 一致）
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "axioma_mpc_controller::MPCControllerPlugin"
      
      # MPC 参数
      prediction_horizon: 10        # 预测时域长度（步数）
      control_horizon: 5             # 控制时域长度
      dt: 0.1                        # 时间步长（秒）
      
      # 权重矩阵
      Q: [10.0, 10.0, 5.0]          # [x, y, θ] 状态权重
      R: [1.0, 1.0]                 # [v, ω] 控制权重
      P: [50.0, 50.0, 20.0]         # 终端权重
      
      # 约束
      max_vel_x: 0.26               # 最大线速度（m/s）
      max_vel_theta: 1.0            # 最大角速度（rad/s）
      min_vel_x: 0.0                # 最小线速度
      acc_lim_x: 2.5                # 加速度限制（m/s²）
      acc_lim_theta: 3.2            # 角加速度限制（rad/s²）
      
      # 障碍物约束
      obstacle_threshold: 50         # 代价地图阈值（0-100）
      obstacle_inflation: 0.2        # 障碍物膨胀半径（米）
      
      # 求解器
      solver: "ipopt"               # 或 "qpoases"
      max_iter: 100                 # 最大迭代次数
      tolerance: 1e-6               # 收敛容差
```

### 7. 预期性能

| 指标 | DWB | MPC |
|------|-----|-----|
| **路径跟踪精度** | 中等（~0.15m） | 高（~0.05m） |
| **速度平滑性** | 中等 | 高 |
| **计算时间** | < 5ms | 10-50ms |
| **实时性** | 20 Hz | 10-20 Hz |
| **避障能力** | 强 | 强（显式约束） |

### 8. 实现时间估算

- **阶段 1（基础MPC）**：2-3天
  - 包创建与接口实现：4小时
  - CasADi 集成：4小时
  - 运动模型实现：2小时
  - 测试与调试：6小时

- **阶段 2（障碍物约束）**：1-2天
  - 代价地图集成：3小时
  - 约束添加：3小时
  - 优化与测试：4小时

- **阶段 3（滑移率）**：3-5天（未来）
  - EKF 实现：2天
  - MPC 扩展：1天
  - 集成测试：1-2天

**总计（阶段1+2）**：3-5天

---

## 下一步行动

1. ✅ **确认需求**：是否只实现基础 MPC（无滑移率）？
2. ✅ **创建包结构**：开始实现 `axioma_mpc_controller`
3. ✅ **实现核心功能**：运动模型 + MPC 优化
4. ✅ **集成测试**：在 Gazebo 中测试

**需要我现在开始实现吗？**
