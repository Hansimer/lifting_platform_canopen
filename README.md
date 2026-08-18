# lifting_platform_canopen — 实时性优化说明

升降平台 CANopen 控制系统。本文档描述**中间层（尽量实时）**与**实时层**的架构、
隔离核（2、5）的使用方式，以及实时性优化与验证方法。

## 1. 分层架构

```
┌──────────────────────────────────────────────────────┐
│                非实时层（MoveIt）                     │
│  moveit_bridge.py（规划轨迹并发送 action goal）       │
│  → /lifting_platform/lifting_platform_controller/     │
│     follow_joint_trajectory                          │
├──────────────────────────────────────────────────────┤
│  中间层（尽量实时）—— 10ms 控制环                     │
│  Joint Trajectory Controller (JTC)                   │
│  ├── 接收 MoveIt 轨迹（action）                       │
│  ├── 按时间戳插值目标位置（10ms，与 CANopen 对齐）    │
│  └── 写入 updown/position 命令接口                    │
│  运行于 controller_manager RT 线程（SCHED_FIFO 85）   │
│  绑定隔离核 5（cpu_affinity=5）                       │
├──────────────────────────────────────────────────────┤
│  实时层 —— CANopen Master / 硬件驱动                  │
│  Cia402System（read/write PDO 数据）                  │
│    + lely master（SYNC 帧发送 / PDO 周期读写）        │
│  CANopen 通信周期 10ms（sync_period=10000µs）         │
│  进程被 taskset -c 2,5 限定在隔离核（lely 落在核 2）  │
└──────────────────────────────────────────────────────┘
```

## 2. 隔离核分配

内核参数（`/proc/cmdline`）已配置：`isolcpus=2,5 nohz_full=2,5 rcu_nocbs=2,5`。

| 核 | 用途 | 调度 |
|----|------|------|
| **5** | controller_manager RT 线程：JTC 插值 + Cia402System 读写（`cpu_affinity: 5`） | SCHED_FIFO 85 |
| **2** | lely CANopen master（SYNC/PDO 交换）及进程内其他线程 | SCHED_OTHER（隔离核，无时钟中断干扰） |
| 0,1,3,4,6,7 | 非隔离核：MoveIt、RViz、ROS 基础设施、CAN IRQ | 默认 |

`launch/canopen_bringup.launch.py` 中 `ros2_control_node` 以
`taskset -c 2,5` 前缀启动，将整个进程（含 lely master 线程）限制在隔离核上，
与 RT 控制环（核 5）互不竞争。

## 3. 实时性关键配置

| 文件 | 参数 | 值 | 说明 |
|------|------|-----|------|
| `config/ros2_controllers.yaml` | `update_rate` | 100 Hz | 10ms 控制环，与 CANopen 10ms 对齐 |
| `config/ros2_controllers.yaml` | `thread_priority` | 85 | RT 线程 SCHED_FIFO |
| `config/ros2_controllers.yaml` | `cpu_affinity` | 5 | RT 线程绑隔离核 5 |
| `config/lifting_platform/bus.yml` | `sync_period` | 10000 µs | CANopen SYNC 10ms（保持不变） |
| `config/lifting_platform/bus.yml` | `0x1006` | 10000 µs | 从站通信周期 |
| `config/lifting_platform/bus.yml` | `defaults.period` | 10 ms | Cia402 驱动处理周期 |

> 说明：`update_rate`（控制环插值频率）与 `sync_period`（总线通信周期）是两个
> 不同层面的时钟。Cia402System::write() 只将目标写入驱动内部缓冲，真正的 RPDO
> 由 lely 在 SYNC(10ms) 时发送，因此提高插值频率不会刷爆总线。当前保持 100Hz
> 与 10ms 完全对齐，如需更平滑插值可调高 `update_rate`（如 500/1000Hz）。

## 4. 中间层（JTC）说明

- JTC（`lifting_platform_controller`）已由 launch 加载（`--inactive`），提供
  `/lifting_platform/lifting_platform_controller/follow_joint_trajectory`
  action 服务，激活后即可接收 MoveIt 轨迹。
- `forward_command_controller`（`lifting_forward_position_controller`）默认激活，
  用于手动位置控制；它与 JTC 共用 `updown/position` 命令接口，**不能同时激活**。
- 控制器切换方法：

```bash
# 默认状态：forward 激活（手动控制），JTC 已加载未激活
# 发目标位置（手动控制）
ros2 topic pub /lifting_forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.3]}"

# 切到 JTC（执行 MoveIt 轨迹）：先停 forward，再激活 JTC
ros2 run controller_manager deactivate_controller \
  lifting_forward_position_controller --controller-manager /lifting_platform/controller_manager
ros2 run controller_manager activate_controller \
  lifting_platform_controller --controller-manager /lifting_platform/controller_manager

# 切回 forward：先停 JTC，再激活 forward
ros2 run controller_manager deactivate_controller \
  lifting_platform_controller --controller-manager /lifting_platform/controller_manager
ros2 run controller_manager activate_controller \
  lifting_forward_position_controller --controller-manager /lifting_platform/controller_manager
```

- `moveit_bridge.py` 调用 MoveIt 规划（plan_only），修复轨迹时间戳后直接发送
  给 JTC 执行（需先切换到 JTC 激活）。

## 5. 运行步骤

```bash
# 1. 配置 CAN 接口（每次开机，需 root）
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# 2. 启动 CANopen 系统（控制器 + CSP 初始化；JTC 以 inactive 加载，forward 激活）
ros2 launch lifting_platform_canopen canopen_bringup.launch.py can_interface_name:=can0

# 3. 手动控制（forward 默认激活）
ros2 topic pub /lifting_forward_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.3]}"

# 4. MoveIt 轨迹执行（先切到 JTC 激活）
ros2 run controller_manager deactivate_controller \
  lifting_forward_position_controller --controller-manager /lifting_platform/controller_manager
ros2 run controller_manager activate_controller \
  lifting_platform_controller --controller-manager /lifting_platform/controller_manager
ros2 launch lifting_platform_moveit_config moveit_rviz.launch.py
ros2 run lifting_platform_canopen moveit_bridge.py --ros-args -p target:=0.3 -p duration:=8.0
```

## 6. 备注

- 内核为 `preempt=full`（5.15.148-tegra），近似 FULL_PREEMPT，非 PREEMPT_RT。
  若需硬实时内核，请刷写带 `CONFIG_PREEMPT_RT` 的内核后重新验证。
- CAN 位速率 500kbps；AIMtor 驱动器支持 1Mbps，如需更高总线裕量可改为 1Mbps
  并同步驱动器配置。
