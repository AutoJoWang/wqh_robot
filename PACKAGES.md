# Axioma 机器人包功能说明文档

本文档详细说明 Axioma 机器人工作空间中各个 ROS 2 包的功能、结构和使用方法。

---

## 目录

- [包概览](#包概览)
- [详细包说明](#详细包说明)
  - [1. axioma_description](#1-axioma_description)
  - [2. axioma_gazebo](#2-axioma_gazebo)
  - [3. axioma_slam](#3-axioma_slam)
  - [4. axioma_navigation](#4-axioma_navigation)
  - [5. axioma_teleop_gui](#5-axioma_teleop_gui)
  - [6. axioma_bringup](#6-axioma_bringup)
- [使用场景](#使用场景)
- [包依赖关系](#包依赖关系)
- [常见问题](#常见问题)

---

## 包概览

| 包名 | 功能 | 构建类型 | 主要组件 |
|------|------|----------|----------|
| `axioma_description` | 机器人模型定义 | ament_cmake | URDF、网格文件、RViz 配置 |
| `axioma_gazebo` | 仿真环境 | ament_python | Gazebo 世界、SDF 模型、ROS-Gazebo 桥接 |
| `axioma_slam` | SLAM 建图 | ament_python | SLAM Toolbox 配置、建图启动文件 |
| `axioma_navigation` | 自主导航 | ament_cmake | Nav2 配置、地图文件、导航启动文件 |
| `axioma_teleop_gui` | 遥控 GUI | ament_python | PyQt5 界面、多种控制模式 |
| `axioma_bringup` | 启动编排 | ament_python | 高级启动文件组合 |

---

## 详细包说明

### 1. axioma_description

**功能**：定义 Axioma 机器人的物理模型和可视化配置

**目录结构**：
```
axioma_description/
├── urdf/
│   └── axioma.urdf          # 机器人 URDF 模型文件
├── meshes/
│   ├── visual/              # 可视化网格文件（.stl）
│   └── collision/           # 碰撞检测网格文件（.dae, .stl）
├── rviz/
│   └── display.rviz        # RViz 可视化配置文件
└── launch/
    └── display.launch.py    # 机器人模型显示启动文件
```

**主要功能**：
- 定义机器人链接（base_link、4 个轮子等）
- 定义关节和运动学约束
- 提供 3D 网格模型用于可视化和碰撞检测
- 配置惯性参数和质量属性

**Launch 文件：`display.launch.py`**

**功能**：启动机器人模型可视化，不包含仿真

**启动的节点**：
- `robot_state_publisher`：发布机器人 TF 树
- `joint_state_publisher_gui`：GUI 界面控制关节状态（用于测试）
- `rviz2`：3D 可视化工具

**使用方法**：
```bash
ros2 launch axioma_description display.launch.py
```

**参数**：
- `use_sim_time` (默认: `false`)：是否使用仿真时间
- `gui` (默认: `true`)：是否启动关节状态发布器 GUI

**使用场景**：
- 查看机器人模型
- 测试 URDF 文件是否正确
- 调试机器人 TF 树
- 不需要仿真时快速查看模型

---

### 2. axioma_gazebo

**功能**：提供 Gazebo/Ignition 仿真环境和 ROS-Gazebo 桥接

**目录结构**：
```
axioma_gazebo/
├── axioma_gazebo/
│   └── odom_to_tf.py        # 里程计转 TF 节点
├── models/
│   └── axioma_v2/           # Gazebo SDF 模型
│       ├── model.sdf        # 机器人 SDF 模型
│       └── meshes/         # 模型网格文件
├── worlds/
│   ├── empty.world          # 空世界
│   └── garage.world         # 车库场景
└── launch/
    ├── simulation.launch.py # 完整仿真启动文件
    └── teleop.launch.py     # 手柄遥控启动文件
```

**主要功能**：
- 启动 Gazebo Harmonic 仿真器
- 在仿真环境中生成机器人
- 桥接 ROS 2 和 Gazebo 之间的通信
- 将 Gazebo 里程计转换为 ROS TF

**Launch 文件：`simulation.launch.py`**

**功能**：启动完整的仿真环境

**启动的组件**：
1. **Gz Sim**：Gazebo 仿真器（服务器 + GUI）
2. **spawn_entity**：在仿真中生成机器人模型
3. **ros_gz_bridge**：ROS-Gazebo 话题桥接
   - `/cmd_vel`：速度控制命令
   - `/odom`：里程计信息
   - `/scan`：激光雷达扫描数据
   - `/clock`：仿真时钟
   - `/joint_states`：关节状态
4. **robot_state_publisher**：发布机器人 TF 树
5. **odom_to_tf**：将 `/odom` 转换为 `odom->base_link` TF

**使用方法**：
```bash
ros2 launch axioma_gazebo simulation.launch.py
```

**参数**：
- `use_sim_time` (默认: `true`)：使用仿真时间
- `world` (默认: `empty.world`)：选择仿真世界文件

**使用场景**：
- 作为其他启动文件的基础（被 bringup 文件调用）
- 单独测试仿真环境
- 调试传感器数据

**Launch 文件：`teleop.launch.py`**

**功能**：启动手柄遥控功能

**启动的节点**：
- `joy_node`：读取手柄输入（`/dev/input/js0`）
- `teleop_twist_joy`：将手柄输入转换为 `Twist` 消息

**使用方法**：
```bash
ros2 launch axioma_gazebo teleop.launch.py
```

**手柄配置**：
- 线性速度轴：轴 1（左摇杆上下）
- 角速度轴：轴 0（左摇杆左右）
- 线性速度缩放：0.5 m/s
- 角速度缩放：2.0 rad/s

---

### 3. axioma_slam

**功能**：SLAM（同时定位与建图）配置和启动文件

**目录结构**：
```
axioma_slam/
├── config/
│   └── slam_params.yaml     # SLAM Toolbox 参数配置
├── rviz/
│   └── slam.rviz           # SLAM 可视化配置
└── launch/
    ├── slam.launch.py      # SLAM 建图启动文件
    └── save_map.launch.py  # 保存地图启动文件
```

**主要功能**：
- 配置 SLAM Toolbox（异步模式）
- 实时构建 2D 占用栅格地图
- 提供地图保存功能

**Launch 文件：`slam.launch.py`**

**功能**：启动 SLAM 建图节点

**启动的节点**：
- `slam_toolbox` (async_slam_toolbox_node)：异步 SLAM 节点

**使用方法**：
```bash
ros2 launch axioma_slam slam.launch.py
```

**参数**：
- `use_sim_time` (默认: `true`)：使用仿真时间
- `params_file`：SLAM 参数文件路径

**工作原理**：
- 订阅 `/scan`（激光雷达数据）和 `/odom`（里程计数据）
- 实时构建地图并发布到 `/map` 话题
- 使用图优化和回环检测提高地图质量

**Launch 文件：`save_map.launch.py`**

**功能**：保存当前构建的地图

**启动的节点**：
- `map_saver_cli`：地图保存工具

**使用方法**：
```bash
ros2 launch axioma_slam save_map.launch.py
```

**参数**：
- `map_path` (默认: `axioma_navigation/maps/mapa`)：保存路径（不含扩展名）

**输出文件**：
- `mapa.pgm`：地图图像文件
- `mapa.yaml`：地图元数据文件

**注意事项**：
- 保存地图前需要先运行 SLAM 建图
- 地图会保存到 `axioma_navigation/maps/` 目录
- 保存后可用于导航

---

### 4. axioma_navigation

**功能**：Navigation2 自主导航配置

**目录结构**：
```
axioma_navigation/
├── config/
│   └── nav2_params.yaml    # Nav2 完整参数配置
├── maps/
│   ├── mapa.pgm            # 已保存的地图图像
│   └── mapa.yaml           # 地图元数据
├── rviz/
│   └── navigation.rviz     # 导航可视化配置
└── launch/
    └── navigation.launch.py # 导航启动文件
```

**主要功能**：
- 配置完整的 Navigation2 栈
- 提供预建地图文件
- 集成 AMCL 定位、路径规划、局部控制

**Launch 文件：`navigation.launch.py`**

**功能**：启动完整的 Navigation2 导航栈

**启动的组件**（通过 nav2_bringup）：
- **Map Server**：加载和发布地图
- **AMCL**：自适应蒙特卡洛定位
- **Planner Server**：全局路径规划（NavFn/Dijkstra）
- **Controller Server**：局部路径跟踪（DWB）
- **Recovery Server**：恢复行为（旋转、后退、等待）
- **BT Navigator**：行为树导航器
- **Lifecycle Manager**：生命周期管理

**使用方法**：
```bash
ros2 launch axioma_navigation navigation.launch.py
```

**参数**：
- `use_sim_time` (默认: `true`)：使用仿真时间
- `autostart` (默认: `true`)：自动启动导航栈
- `params_file`：Nav2 参数文件路径
- `map`：地图 YAML 文件路径

**使用流程**：
1. 在 RViz 中使用 **2D Pose Estimate** 设置机器人初始位姿
2. 等待 AMCL 粒子收敛（观察粒子云）
3. 使用 **2D Goal Pose** 设置导航目标
4. 机器人自动规划路径并导航到目标

**可视化内容**：
- 全局代价地图（静态障碍物）
- 局部代价地图（动态障碍物）
- 全局路径（粗线）
- 局部路径（细线）
- AMCL 粒子云

---

### 5. axioma_teleop_gui

**功能**：基于 PyQt5 的图形化遥控界面

**目录结构**：
```
axioma_teleop_gui/
├── axioma_teleop_gui/
│   ├── main.py              # 主程序入口
│   ├── main_window.py       # 主窗口界面
│   ├── ros_node.py          # ROS 2 节点（发布 cmd_vel）
│   └── widgets/
│       ├── keyboard_mode.py # 键盘控制模式
│       ├── joystick_mode.py # 虚拟摇杆模式
│       └── slider_mode.py   # 滑块控制模式
└── launch/
    └── teleop_gui.launch.py # GUI 启动文件
```

**主要功能**：
- 提供三种控制模式：键盘、虚拟摇杆、滑块
- 实时显示速度值
- 线程安全的 ROS 2 通信
- 可切换控制话题

**Launch 文件：`teleop_gui.launch.py`**

**功能**：启动遥控 GUI 界面

**启动的节点**：
- `teleop_gui`：PyQt5 GUI 应用程序

**使用方法**：
```bash
ros2 launch axioma_teleop_gui teleop_gui.launch.py
```

**控制模式**：

1. **键盘模式**：
   - `W/S`：前进/后退
   - `A/D`：左转/右转
   - `Space`：停止

2. **虚拟摇杆模式**：
   - 鼠标拖动虚拟摇杆
   - 中心为停止，边缘为最大速度

3. **滑块模式**：
   - 线性速度滑块（-1.0 到 1.0）
   - 角速度滑块（-1.0 到 1.0）

**发布的话题**：
- `/cmd_vel` (geometry_msgs/Twist)：速度控制命令

**特性**：
- 15 Hz 发布频率
- 线程安全的速度更新
- 实时速度显示
- 可切换控制话题（运行时）

---

### 6. axioma_bringup

**功能**：高级启动编排，组合多个包实现完整功能

**目录结构**：
```
axioma_bringup/
├── launch/
│   ├── slam_bringup.launch.py      # SLAM 建图完整启动
│   └── navigation_bringup.launch.py # 导航完整启动
└── axioma_bringup/
    └── __init__.py
```

**主要功能**：
- 提供一键启动完整功能场景
- 协调多个包的启动顺序
- 统一配置 RViz 可视化

**Launch 文件：`slam_bringup.launch.py`**

**功能**：一键启动仿真 + SLAM 建图

**启动的组件**：
1. **仿真环境**（`axioma_gazebo/simulation.launch.py`）
2. **SLAM 建图**（`axioma_slam/slam.launch.py`）
3. **RViz 可视化**（SLAM 配置）

**使用方法**：
```bash
ros2 launch axioma_bringup slam_bringup.launch.py
```

**完整工作流程**：
1. 启动后，Gazebo 仿真器打开，机器人出现在空世界中
2. RViz 打开，显示激光扫描和实时构建的地图
3. 在另一个终端启动遥控：
   ```bash
   ros2 launch axioma_teleop_gui teleop_gui.launch.py
   # 或
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
4. 控制机器人移动，观察地图实时构建
5. 建图完成后，保存地图：
   ```bash
   ros2 launch axioma_slam save_map.launch.py
   ```

**参数**：
- `use_sim_time` (默认: `true`)：使用仿真时间

**Launch 文件：`navigation_bringup.launch.py`**

**功能**：一键启动仿真 + 自主导航

**启动的组件**：
1. **仿真环境**（`axioma_gazebo/simulation.launch.py`）
2. **静态 TF 发布器**（`map->odom`，临时，直到 AMCL 初始化）
3. **导航栈**（`axioma_navigation/navigation.launch.py`）
4. **RViz 可视化**（导航配置）

**使用方法**：
```bash
ros2 launch axioma_bringup navigation_bringup.launch.py
```

**完整工作流程**：
1. 启动后，Gazebo 仿真器打开，机器人出现在空世界中
2. RViz 打开，显示地图、代价地图和导航信息
3. **设置初始位姿**：在 RViz 中使用 "2D Pose Estimate" 工具，在地图上点击并拖动设置机器人初始位置和朝向
4. 等待 AMCL 粒子收敛（观察粒子云分布）
5. **设置导航目标**：使用 "2D Goal Pose" 工具，在地图上点击设置目标位置
6. 机器人自动规划路径并导航到目标

**参数**：
- `use_sim_time` (默认: `true`)：使用仿真时间

**注意事项**：
- 需要预先保存的地图文件（`axioma_navigation/maps/mapa.yaml`）
- 如果地图不存在，需要先运行 SLAM 建图并保存地图

---

## 使用场景

### 场景 1：建图（Mapping）

**目标**：构建环境地图

**步骤**：
```bash
# 1. 启动仿真和 SLAM
ros2 launch axioma_bringup slam_bringup.launch.py

# 2. 在另一个终端启动遥控
ros2 launch axioma_teleop_gui teleop_gui.launch.py

# 3. 控制机器人探索环境，观察地图实时构建

# 4. 建图完成后保存地图
ros2 launch axioma_slam save_map.launch.py
```

**预期结果**：
- Gazebo 中机器人移动
- RViz 中实时显示构建的地图
- 地图保存到 `axioma_navigation/maps/`

---

### 场景 2：自主导航（Autonomous Navigation）

**目标**：机器人自主导航到指定位置

**前提条件**：
- 已有保存的地图文件

**步骤**：
```bash
# 1. 启动仿真和导航
ros2 launch axioma_bringup navigation_bringup.launch.py

# 2. 在 RViz 中设置初始位姿（2D Pose Estimate）

# 3. 在 RViz 中设置导航目标（2D Goal Pose）

# 4. 观察机器人自主导航
```

**预期结果**：
- 机器人自动规划路径
- 避开障碍物
- 到达目标位置

---

### 场景 3：查看机器人模型

**目标**：仅查看机器人 3D 模型，不启动仿真

**步骤**：
```bash
ros2 launch axioma_description display.launch.py
```

**预期结果**：
- RViz 打开，显示机器人模型
- 可以通过 GUI 调整关节状态
- 无仿真环境

---

### 场景 4：仅仿真环境

**目标**：仅启动仿真，不启动 SLAM 或导航

**步骤**：
```bash
ros2 launch axioma_gazebo simulation.launch.py
```

**预期结果**：
- Gazebo 仿真器打开
- 机器人出现在仿真世界中
- 可以手动控制机器人移动
- 无 SLAM 或导航功能

---

## 包依赖关系

```
axioma_bringup (顶层编排)
    │
    ├── axioma_gazebo (仿真环境)
    │   ├── axioma_description (机器人模型)
    │   └── ros_gz_sim, ros_gz_bridge
    │
    ├── axioma_slam (建图)
    │   └── slam_toolbox
    │
    └── axioma_navigation (导航)
        ├── navigation2, nav2_bringup
        └── 使用 axioma_slam 生成的地图

axioma_teleop_gui (独立包，可配合任何场景使用)
    └── rclpy, geometry_msgs
```

**依赖说明**：
- `axioma_bringup` 依赖所有其他包（除了 `axioma_teleop_gui`）
- `axioma_gazebo` 依赖 `axioma_description`（需要 URDF 模型）
- `axioma_navigation` 需要预先保存的地图（通常由 `axioma_slam` 生成）
- `axioma_teleop_gui` 是独立的，可以在任何场景中使用

---

## 常见问题

### Q1: 启动 SLAM 后看不到地图？

**A**: 检查以下几点：
- 确保激光雷达数据正常：`ros2 topic echo /scan`
- 确保里程计数据正常：`ros2 topic echo /odom`
- 检查 SLAM 节点是否运行：`ros2 node list | grep slam`
- 在 RViz 中添加 Map 显示，话题选择 `/map`

### Q2: 导航时机器人不移动？

**A**: 检查以下几点：
- 是否设置了初始位姿（2D Pose Estimate）？
- AMCL 粒子是否收敛（观察粒子云）？
- 检查 `/cmd_vel` 话题是否有数据：`ros2 topic echo /cmd_vel`
- 检查控制器服务器状态：`ros2 lifecycle get /controller_server`

### Q3: 地图保存失败？

**A**: 检查以下几点：
- 确保 SLAM 正在运行
- 检查地图话题是否有数据：`ros2 topic echo /map`
- 检查保存路径是否有写权限
- 确保 `map_saver` 节点正常启动

### Q4: 仿真中机器人不响应控制命令？

**A**: 检查以下几点：
- 检查 `/cmd_vel` 话题：`ros2 topic echo /cmd_vel`
- 检查 ros_gz_bridge 是否正常运行
- 检查 Gazebo 中机器人模型是否正确生成
- 查看 bridge 节点日志

### Q5: TF 树错误？

**A**: 检查以下几点：
- 查看 TF 树：`ros2 run tf2_tools view_frames`
- 检查 `odom_to_tf` 节点是否运行
- 检查 `robot_state_publisher` 是否运行
- 验证 TF 关系：`ros2 run tf2_ros tf2_echo map base_link`

### Q6: 如何切换到真实机器人？

**A**: 
- 修改 launch 文件中的 `use_sim_time` 参数为 `false`
- 确保真实机器人的传感器话题名称匹配（`/scan`, `/odom` 等）
- 不需要启动 `axioma_gazebo` 相关文件
- 直接启动 `axioma_slam` 或 `axioma_navigation`

---

## 总结

Axioma 机器人工作空间提供了完整的 ROS 2 移动机器人解决方案：

- **仿真**：Gazebo Harmonic 完整仿真环境
- **建图**：SLAM Toolbox 实时建图
- **导航**：Navigation2 自主导航
- **控制**：多种遥控方式（GUI、键盘、手柄）
- **可视化**：RViz2 完整可视化

通过 `axioma_bringup` 包的高级启动文件，可以一键启动完整功能场景，大大简化了使用流程。

---

**文档版本**：1.0  
**最后更新**：2024  
**维护者**：Mario David Alvarez Vallejo
