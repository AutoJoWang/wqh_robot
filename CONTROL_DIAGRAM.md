# Axioma 机器人系统控制框图

本文档基于现有代码实现，详细描述了 Axioma 机器人的完整控制架构和数据流。

---

## 系统总体架构

```mermaid
flowchart TB
    subgraph UserLayer["用户输入层"]
        RViz["RViz2<br/>2D Goal Pose<br/>2D Pose Estimate"]
        TeleopGUI["Teleop GUI<br/>PyQt5 Interface"]
        Keyboard["Keyboard Teleop"]
    end

    subgraph Nav2Layer["Navigation2 导航层"]
        BTNav["BT Navigator<br/>行为树导航器<br/>20 Hz"]
        Planner["Planner Server<br/>NavFn/Dijkstra<br/>全局路径规划"]
        Controller["Controller Server<br/>DWB Local Planner<br/>20 Hz"]
        AMCL["AMCL<br/>自适应蒙特卡洛定位<br/>粒子滤波"]
        MapServer["Map Server<br/>地图加载"]
        CostmapGlobal["Global Costmap<br/>全局代价地图<br/>1 Hz"]
        CostmapLocal["Local Costmap<br/>局部代价地图<br/>5 Hz"]
        Smoother["Velocity Smoother<br/>速度平滑<br/>20 Hz"]
        Behavior["Behavior Server<br/>恢复行为<br/>10 Hz"]
    end

    subgraph SimLayer["仿真层 (Gazebo Harmonic)"]
        GzSim["Gazebo Sim<br/>物理仿真引擎"]
        GzBridge["ros_gz_bridge<br/>ROS-Gazebo 桥接"]
        DiffDrive["Diff Drive Plugin<br/>差速驱动插件<br/>50 Hz"]
        JointPub["Joint State Publisher<br/>关节状态发布<br/>20 Hz"]
    end

    subgraph SensorLayer["传感器层"]
        LiDAR["LiDAR Sensor<br/>RPLidar A1<br/>360° 扫描<br/>5.5 Hz"]
        Odom["Odometry<br/>里程计<br/>50 Hz"]
        JointStates["Joint States<br/>4个轮子状态<br/>20 Hz"]
    end

    subgraph StateLayer["状态估计层"]
        OdomToTF["odom_to_tf<br/>里程计转TF"]
        RobotStatePub["robot_state_publisher<br/>机器人状态发布<br/>URDF → TF"]
        StaticTF["static_transform_publisher<br/>map → odom<br/>临时TF"]
    end

    subgraph TFLayer["TF 树"]
        MapFrame["map"]
        OdomFrame["odom"]
        BaseFrame["base_link"]
        ScanFrame["base_scan"]
        ImuFrame["imu_link"]
    end

    subgraph VisLayer["可视化层"]
        RVizVis["RViz2<br/>可视化工具"]
    end

    %% 用户输入流
    RViz -->|"/goal_pose<br/>Action"| BTNav
    TeleopGUI -->|"/cmd_vel<br/>Twist"| GzBridge
    Keyboard -->|"/cmd_vel<br/>Twist"| GzBridge

    %% Nav2 内部流
    BTNav -->|"导航任务"| Planner
    BTNav -->|"控制任务"| Controller
    BTNav -->|"恢复任务"| Behavior
    Planner -->|"/plan<br/>Path"| Controller
    MapServer -->|"/map<br/>OccupancyGrid"| Planner
    MapServer -->|"/map"| CostmapGlobal
    AMCL -->|"/amcl_pose<br/>PoseWithCovariance"| Planner
    AMCL -->|"map → odom<br/>TF"| TFLayer
    CostmapGlobal -->|"全局障碍物"| Planner
    CostmapLocal -->|"局部障碍物"| Controller
    Controller -->|"/cmd_vel<br/>Twist"| Smoother
    Smoother -->|"/cmd_vel<br/>Twist"| GzBridge

    %% 传感器数据流
    LiDAR -->|"/scan<br/>LaserScan"| GzBridge
    Odom -->|"/odom<br/>Odometry"| GzBridge
    JointStates -->|"/joint_states<br/>JointState"| GzBridge

    %% Gazebo 桥接
    GzBridge -->|"/scan"| CostmapLocal
    GzBridge -->|"/scan"| CostmapGlobal
    GzBridge -->|"/scan"| AMCL
    GzBridge -->|"/odom"| AMCL
    GzBridge -->|"/odom"| OdomToTF
    GzBridge -->|"/odom"| Smoother
    GzBridge -->|"/joint_states"| RobotStatePub
    GzBridge -->|"/cmd_vel"| DiffDrive

    %% 仿真物理
    DiffDrive -->|"轮子速度"| GzSim
    GzSim -->|"物理响应"| LiDAR
    GzSim -->|"物理响应"| Odom
    GzSim -->|"物理响应"| JointStates

    %% 状态估计流
    OdomToTF -->|"odom → base_link<br/>TF"| TFLayer
    RobotStatePub -->|"base_link → *<br/>TF树"| TFLayer
    StaticTF -->|"map → odom<br/>TF"| TFLayer

    %% TF 树结构
    MapFrame --> OdomFrame
    OdomFrame --> BaseFrame
    BaseFrame --> ScanFrame
    BaseFrame --> ImuFrame

    %% 可视化
    TFLayer --> RVizVis
    CostmapGlobal --> RVizVis
    CostmapLocal --> RVizVis
    Planner -->|"/plan"| RVizVis
    AMCL -->|"粒子云"| RVizVis

    style UserLayer fill:#e1f5ff
    style Nav2Layer fill:#fff4e1
    style SimLayer fill:#e8f5e9
    style SensorLayer fill:#fce4ec
    style StateLayer fill:#f3e5f5
    style TFLayer fill:#fff9c4
    style VisLayer fill:#e0f2f1
```

---

## 详细数据流图

### 1. 自主导航控制流

```mermaid
sequenceDiagram
    participant User as 用户 (RViz)
    participant BT as BT Navigator
    participant Planner as Planner Server
    participant Controller as Controller Server
    participant Smoother as Velocity Smoother
    participant Bridge as ros_gz_bridge
    participant Plugin as Diff Drive Plugin
    participant Robot as Gazebo 机器人

    User->>BT: 设置导航目标 (2D Goal Pose)
    BT->>Planner: 请求全局路径
    Planner->>Planner: 规划路径 (NavFn/Dijkstra)
    Planner->>BT: 返回全局路径
    BT->>Controller: 执行路径跟踪
    loop 20 Hz 控制循环
        Controller->>Controller: DWB 局部规划
        Controller->>Smoother: 发送速度命令
        Smoother->>Smoother: 速度平滑处理
        Smoother->>Bridge: /cmd_vel (Twist)
        Bridge->>Plugin: 转换为 Gazebo 消息
        Plugin->>Plugin: 差速驱动计算
        Plugin->>Robot: 应用轮子速度
        Robot->>Robot: 物理仿真
    end
```

### 2. 传感器数据流

```mermaid
flowchart LR
    subgraph Gazebo["Gazebo 仿真环境"]
        Physics["物理引擎"]
        LiDARSim["LiDAR 传感器<br/>模拟"]
        OdomSim["里程计计算<br/>50 Hz"]
        JointSim["关节状态<br/>4个轮子"]
    end

    subgraph Bridge["ROS-Gazebo 桥接"]
        BridgeNode["parameter_bridge"]
    end

    subgraph ROS2["ROS 2 系统"]
        ScanTopic["/scan<br/>LaserScan"]
        OdomTopic["/odom<br/>Odometry"]
        JointTopic["/joint_states<br/>JointState"]
    end

    subgraph Consumers["数据消费者"]
        AMCLNode["AMCL<br/>定位"]
        CostmapNode["Costmap<br/>障碍物检测"]
        OdomTFNode["odom_to_tf<br/>TF转换"]
        StatePubNode["robot_state_publisher<br/>TF发布"]
    end

    Physics --> LiDARSim
    Physics --> OdomSim
    Physics --> JointSim

    LiDARSim -->|"激光数据"| BridgeNode
    OdomSim -->|"里程计数据"| BridgeNode
    JointSim -->|"关节数据"| BridgeNode

    BridgeNode -->|"转换"| ScanTopic
    BridgeNode -->|"转换"| OdomTopic
    BridgeNode -->|"转换"| JointTopic

    ScanTopic --> AMCLNode
    ScanTopic --> CostmapNode
    OdomTopic --> AMCLNode
    OdomTopic --> OdomTFNode
    JointTopic --> StatePubNode
```

### 3. 状态估计和定位流

```mermaid
flowchart TB
    subgraph Sensors["传感器输入"]
        Scan["/scan<br/>LaserScan<br/>5.5 Hz"]
        Odom["/odom<br/>Odometry<br/>50 Hz"]
        Map["/map<br/>OccupancyGrid<br/>静态地图"]
    end

    subgraph AMCLProcess["AMCL 定位流程"]
        MotionModel["运动模型<br/>DifferentialMotionModel<br/>α1-α5: 0.05"]
        ParticleFilter["粒子滤波<br/>5000 粒子<br/>1000-5000 范围"]
        ScanMatch["扫描匹配<br/>Likelihood Field<br/>120 beams"]
        Resample["重采样<br/>每1步"]
    end

    subgraph Output["输出"]
        AMCLPose["/amcl_pose<br/>PoseWithCovariance"]
        AMCLTF["map → odom<br/>TF变换"]
    end

    Scan --> ScanMatch
    Odom --> MotionModel
    Map --> ScanMatch

    MotionModel --> ParticleFilter
    ScanMatch --> ParticleFilter
    ParticleFilter --> Resample
    Resample --> ParticleFilter

    ParticleFilter --> AMCLPose
    ParticleFilter --> AMCLTF
```

### 4. 局部路径跟踪控制流 (DWB)

```mermaid
flowchart TB
    subgraph Input["输入"]
        GlobalPath["全局路径<br/>/plan<br/>Path"]
        RobotPose["机器人位姿<br/>TF: base_link"]
        Costmap["局部代价地图<br/>3m × 3m<br/>0.05m 分辨率"]
        RobotVel["当前速度<br/>/odom"]
    end

    subgraph DWB["DWB Local Planner"]
        TrajGen["轨迹生成<br/>vx: 20 samples<br/>vy: 5 samples<br/>vtheta: 20 samples"]
        TrajSim["轨迹仿真<br/>sim_time: 1.7s<br/>预测未来轨迹"]
        Critics["评价函数<br/>7个critics"]
        Select["选择最优轨迹"]
    end

    subgraph CriticsDetail["评价函数详情"]
        PathAlign["PathAlign<br/>路径对齐<br/>scale: 32.0"]
        GoalAlign["GoalAlign<br/>目标对齐<br/>scale: 24.0"]
        PathDist["PathDist<br/>路径距离<br/>scale: 32.0"]
        GoalDist["GoalDist<br/>目标距离<br/>scale: 24.0"]
        BaseObstacle["BaseObstacle<br/>障碍物避让<br/>scale: 0.02"]
        RotateToGoal["RotateToGoal<br/>旋转到目标<br/>scale: 32.0"]
        Oscillation["Oscillation<br/>振荡检测"]
    end

    subgraph Output["输出"]
        CmdVel["/cmd_vel<br/>Twist<br/>max: 0.26 m/s, 1.0 rad/s"]
    end

    GlobalPath --> TrajGen
    RobotPose --> TrajGen
    Costmap --> TrajGen
    RobotVel --> TrajGen

    TrajGen --> TrajSim
    TrajSim --> Critics

    Critics --> PathAlign
    Critics --> GoalAlign
    Critics --> PathDist
    Critics --> GoalDist
    Critics --> BaseObstacle
    Critics --> RotateToGoal
    Critics --> Oscillation

    PathAlign --> Select
    GoalAlign --> Select
    PathDist --> Select
    GoalDist --> Select
    BaseObstacle --> Select
    RotateToGoal --> Select
    Oscillation --> Select

    Select --> CmdVel
```

### 5. TF 树结构

```mermaid
graph TD
    Map["base_link<br/>(全局坐标系)"]
    Odom["odom<br/>(里程计坐标系)"]
    BaseLink["base_link<br/>(机器人基座)"]
    BaseScan["base_scan<br/>(激光雷达)"]
    ImuLink["imu_link<br/>(IMU传感器)"]
    Wheel1["wheel_1<br/>(前左轮)"]
    Wheel2["wheel_2<br/>(后左轮)"]
    Wheel3["wheel_3<br/>(后右轮)"]
    Wheel4["wheel_4<br/>(前右轮)"]

    Map -->|"AMCL 发布<br/>或 static_transform"| Odom
    Odom -->|"odom_to_tf 发布<br/>从 /odom 话题"| BaseLink
    BaseLink -->|"robot_state_publisher<br/>从 URDF"| BaseScan
    BaseLink -->|"robot_state_publisher<br/>从 URDF"| ImuLink
    BaseLink -->|"robot_state_publisher<br/>从 URDF"| Wheel1
    BaseLink -->|"robot_state_publisher<br/>从 URDF"| Wheel2
    BaseLink -->|"robot_state_publisher<br/>从 URDF"| Wheel3
    BaseLink -->|"robot_state_publisher<br/>从 URDF"| Wheel4

    style Map fill:#ffeb3b
    style Odom fill:#4caf50
    style BaseLink fill:#2196f3
    style BaseScan fill:#f44336
    style ImuLink fill:#f44336
```

---

## 关键话题和消息类型

### 控制命令流

| 话题 | 消息类型 | 发布者 | 订阅者 | 频率 | 说明 |
|------|----------|--------|--------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity Smoother / Teleop | ros_gz_bridge | 20 Hz | 速度控制命令 |
| `/plan` | `nav_msgs/Path` | Planner Server | Controller Server | 按需 | 全局路径 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz / BT Navigator | BT Navigator | 按需 | 导航目标 |

### 传感器数据流

| 话题 | 消息类型 | 发布者 | 订阅者 | 频率 | 说明 |
|------|----------|--------|--------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | ros_gz_bridge | AMCL, Costmap | 5.5 Hz | 激光雷达扫描 |
| `/odom` | `nav_msgs/Odometry` | ros_gz_bridge | AMCL, odom_to_tf, Smoother | 50 Hz | 里程计数据 |
| `/joint_states` | `sensor_msgs/JointState` | ros_gz_bridge | robot_state_publisher | 20 Hz | 关节状态（4个轮子） |

### 状态估计流

| 话题 | 消息类型 | 发布者 | 订阅者 | 频率 | 说明 |
|------|----------|--------|--------|------|------|
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL | Planner, RViz | 按需 | AMCL 估计位姿 |
| `/map` | `nav_msgs/OccupancyGrid` | Map Server | Planner, Costmap | 静态 | 占用栅格地图 |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | Local Costmap | Controller, RViz | 2 Hz | 局部代价地图 |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | Global Costmap | Planner, RViz | 1 Hz | 全局代价地图 |

### TF 变换

| 父框架 | 子框架 | 发布者 | 频率 | 说明 |
|--------|--------|--------|------|------|
| `map` | `odom` | AMCL / static_transform | 按需/静态 | 全局定位 |
| `odom` | `base_link` | odom_to_tf | 50 Hz | 里程计位姿 |
| `base_link` | `base_scan` | robot_state_publisher | 50 Hz | 激光雷达位置 |
| `base_link` | `imu_link` | robot_state_publisher | 50 Hz | IMU 位置 |
| `base_link` | `wheel_*` | robot_state_publisher | 50 Hz | 轮子位置 |

---

## 组件详细说明

### Navigation2 栈组件

#### 1. BT Navigator (行为树导航器)
- **功能**：协调整个导航流程
- **频率**：20 Hz
- **输入**：导航目标 (`/goal_pose`)
- **输出**：导航任务到 Planner 和 Controller
- **配置**：`nav2_params.yaml` lines 52-106

#### 2. Planner Server (全局路径规划)
- **功能**：从当前位置到目标位置的全局路径规划
- **算法**：NavFn (Dijkstra)
- **输入**：地图 (`/map`)、当前位姿 (AMCL)、目标位姿
- **输出**：全局路径 (`/plan`)
- **配置**：`nav2_params.yaml` lines 246-255

#### 3. Controller Server (局部路径跟踪)
- **功能**：局部路径跟踪和避障
- **算法**：DWB (Dynamic Window Approach)
- **频率**：20 Hz
- **输入**：全局路径 (`/plan`)、局部代价地图、当前速度
- **输出**：速度命令 (`/cmd_vel`)
- **配置**：`nav2_params.yaml` lines 108-169

#### 4. AMCL (自适应蒙特卡洛定位)
- **功能**：在已知地图中定位机器人
- **算法**：粒子滤波
- **粒子数**：1000-5000
- **输入**：激光扫描 (`/scan`)、里程计 (`/odom`)、地图 (`/map`)
- **输出**：估计位姿 (`/amcl_pose`)、TF (`map → odom`)
- **配置**：`nav2_params.yaml` lines 6-50

#### 5. Costmap (代价地图)
- **Global Costmap**：
  - 范围：整个地图
  - 更新频率：1 Hz
  - 用途：全局路径规划
- **Local Costmap**：
  - 范围：3m × 3m (滚动窗口)
  - 更新频率：5 Hz
  - 用途：局部避障
- **配置**：`nav2_params.yaml` lines 171-244

### 仿真层组件

#### 1. Gazebo Sim
- **功能**：物理仿真引擎
- **世界文件**：`empty.world` 或 `garage.world`
- **物理引擎**：Bullet/ODE

#### 2. ros_gz_bridge
- **功能**：ROS 2 和 Gazebo 之间的消息桥接
- **桥接话题**：
  - `/cmd_vel`: ROS → Gazebo
  - `/odom`: Gazebo → ROS
  - `/scan`: Gazebo → ROS
  - `/joint_states`: Gazebo → ROS
  - `/clock`: Gazebo → ROS
- **配置**：`simulation.launch.py` lines 68-79

#### 3. Diff Drive Plugin
- **功能**：差速驱动控制
- **频率**：50 Hz
- **输入**：`/cmd_vel` (Twist)
- **输出**：轮子速度、里程计 (`/odom`)
- **参数**：
  - 轮子半径：0.0381 m
  - 轮子间距：0.1725 m
  - 最大扭矩：20 N·m
  - 最大加速度：1.0 m/s²
- **配置**：`model.sdf` lines 404-419

### 状态估计组件

#### 1. odom_to_tf
- **功能**：将里程计消息转换为 TF 变换
- **输入**：`/odom` (Odometry)
- **输出**：TF `odom → base_link`
- **实现**：`axioma_gazebo/odom_to_tf.py`

#### 2. robot_state_publisher
- **功能**：根据 URDF 和关节状态发布 TF 树
- **输入**：URDF、`/joint_states`
- **输出**：TF 树 (`base_link → base_scan`, `base_link → imu_link`, `base_link → wheel_*`)

#### 3. static_transform_publisher
- **功能**：发布静态 TF 变换
- **用途**：临时 `map → odom` 变换（直到 AMCL 初始化）
- **配置**：`navigation_bringup.launch.py` lines 35-40

---

## 控制频率总结

| 组件 | 频率 | 说明 |
|------|------|------|
| Diff Drive Plugin | 50 Hz | 最高频率，直接控制轮子 |
| Odometry | 50 Hz | 里程计发布频率 |
| robot_state_publisher | 50 Hz | TF 树更新频率 |
| odom_to_tf | 50 Hz | TF 转换频率 |
| Controller Server | 20 Hz | 局部路径跟踪 |
| Velocity Smoother | 20 Hz | 速度平滑 |
| Joint State Publisher | 20 Hz | 关节状态发布 |
| Local Costmap | 5 Hz | 局部代价地图更新 |
| LiDAR | 5.5 Hz | 激光雷达扫描频率 |
| Global Costmap | 1 Hz | 全局代价地图更新 |
| Behavior Server | 10 Hz | 恢复行为 |
| AMCL | 按需 | 由里程计和扫描触发 |

---

## 数据流路径总结

### 控制命令路径
```
用户输入 (RViz/Teleop)
    ↓
BT Navigator
    ↓
Planner Server → 全局路径 (/plan)
    ↓
Controller Server (DWB)
    ↓
Velocity Smoother
    ↓
ros_gz_bridge
    ↓
Diff Drive Plugin
    ↓
Gazebo 物理引擎
    ↓
机器人运动
```

### 传感器数据路径
```
Gazebo 传感器
    ↓
ros_gz_bridge
    ↓
ROS 2 话题
    ├→ /scan → AMCL, Costmap
    ├→ /odom → AMCL, odom_to_tf, Smoother
    └→ /joint_states → robot_state_publisher
```

### 状态估计路径
```
传感器数据
    ↓
AMCL (粒子滤波)
    ↓
/amcl_pose + TF (map → odom)
    ↓
Planner Server (用于路径规划)
```

### TF 树构建路径
```
URDF + /joint_states
    ↓
robot_state_publisher
    ↓
TF: base_link → base_scan, imu_link, wheel_*

/odom
    ↓
odom_to_tf
    ↓
TF: odom → base_link

AMCL
    ↓
TF: map → odom
```

---

## 关键参数配置位置

| 参数类型 | 配置文件 | 位置 |
|----------|----------|------|
| Nav2 全局配置 | `nav2_params.yaml` | `src/axioma_navigation/config/` |
| DWB Controller | `nav2_params.yaml` | lines 131-169 |
| AMCL 定位 | `nav2_params.yaml` | lines 6-50 |
| Costmap 配置 | `nav2_params.yaml` | lines 171-244 |
| Gazebo 模型 | `model.sdf` | `src/axioma_gazebo/models/axioma_v2/` |
| 机器人 URDF | `axioma.urdf` | `src/axioma_description/urdf/` |
| 启动配置 | `navigation_bringup.launch.py` | `src/axioma_bringup/launch/` |

---

## 系统特性

### 优势
1. **模块化设计**：各组件独立，易于替换和调试
2. **实时性**：控制频率 20-50 Hz，满足实时控制需求
3. **鲁棒性**：多层避障（全局规划 + 局部避障）
4. **可扩展性**：支持插件化控制器（可替换 DWB）

### 限制
1. **无滑移补偿**：当前系统假设无滑移，实际滑移会影响定位精度
2. **无速度闭环**：Gazebo 插件直接应用速度命令，无 PID 控制
3. **固定参数**：所有参数需要手动调优

---

**文档版本**：1.0  
**最后更新**：2024-03-11  
**基于代码版本**：当前工作空间实现
