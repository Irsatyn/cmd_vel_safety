# cmd_vel_safety — 机器人速度指令处理与状态监控系统

ROS 2 Humble / C++17。在不可信的 `/cmd_vel` 生产者与底盘之间插入一道安全网关，
并独立监控运动状态与链路健康。

- **[PROJECT_OUTLINE.md](PROJECT_OUTLINE.md)** — 设计文档（架构、消息、参数、安全流水线、异常矩阵）
- **[docs/scenario.md](docs/scenario.md)** — 给定 rosbag 的逐段标注与**实测结果**

---

## 1. 数据流

```
rosbag / teleop ──/cmd_vel──► velocity_guard ──/cmd_vel_safe──► virtual_robot ──/odom──┐
                       ▲            │                                                  │
                  /e_stop           └──/velocity_guard/report──► motion_state_monitor ◄─┘
                                                                        │
                                   /robot_motion_state  /robot_status  /diagnostics
```

| 节点                   | 职责                                                    |
| ---------------------- | ------------------------------------------------------- |
| `velocity_guard`       | 校验、限幅、斜率限制、失效安全。固定 20 Hz 输出         |
| `motion_state_monitor` | 运动状态分类与链路健康监控。不在控制路径上              |
| `virtual_robot`        | 差速底盘模型（含一阶电机滞后 + 独立看门狗），闭合数据流 |

## 2. 构建

```bash
cd cmd_vel_safety_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. 运行

```bash
# 回放给定 bag，一条命令跑完全部异常场景
ros2 launch cmd_vel_safety replay_bag.launch.py

# 放慢观察 / 循环回放（循环会触发时钟回跳处理）
ros2 launch cmd_vel_safety replay_bag.launch.py rate:=0.3
ros2 launch cmd_vel_safety replay_bag.launch.py loop:=true

# 回放并录制全链路，产出可离线核查的 bag
ros2 launch cmd_vel_safety record_bag.launch.py output:=/tmp/run1

# 带 rqt_graph + rqt_plot 的一键演示（需要图形界面）
ros2 launch cmd_vel_safety demo.launch.py

# 接真实上游（关掉内置底盘模型）
ros2 launch cmd_vel_safety bringup.launch.py enable_virtual_robot:=false
```

人可读状态：

```bash
ros2 topic echo /robot_status
# [FORWARD 2.40s] v=+0.30 m/s  w=+0.00 rad/s  R=inf  | dist=1.85 m
# | cmd_in=10.00 Hz safe_out=20.00 Hz | interventions=14 | OK: nominal
```

## 4. 验证

```bash
# 单元测试：19 项，覆盖每条规则的边界行为
colcon test --packages-select cmd_vel_safety
colcon test-result --verbose
# 或直接运行
./build/cmd_vel_safety/test_safety_limiter

# 离线断言录制结果满足全部安全不变量
ros2 run cmd_vel_safety check_bag.py /tmp/run1
```

实测结论见 [docs/scenario.md](docs/scenario.md)。11 条安全规则全部被实测覆盖：
回放给定 bag 覆盖 8 条，死区 / 持续非法输入 / 急停 3 条由在线测试覆盖。

## 5. 急停

`/e_stop` 订阅端使用 **TransientLocal** QoS，这样「急停已置位时才启动的网关」
依然会保持停止 —— 避免「重启节点 = 解除急停」。

代价是一个 QoS 兼容陷阱：**Volatile 发布者与 TransientLocal 订阅者不兼容，
DDS 会静默地什么都不投递**。裸 `ros2 topic pub` 默认就是 Volatile，因此必须
显式指定：

```bash
# 正确：显式 transient_local，并保持发布者存活（不要加 -1）
ros2 topic pub /e_stop std_msgs/msg/Bool '{data: true}' \
  --qos-durability transient_local --qos-reliability reliable

# 解除
ros2 topic pub /e_stop std_msgs/msg/Bool '{data: false}' \
  --qos-durability transient_local --qos-reliability reliable
```

若上游无法提供 transient_local，可设 `estop_qos_durability:=volatile`
接受任意发布者，代价是失去「启动即保持急停」的能力。

## 6. rqt

```bash
rqt_graph        # 核对三节点 / 8 个 topic 的单向数据流
rqt_plot /cmd_vel/linear/x /cmd_vel_safe/linear/x /velocity_guard/report/flags
rqt_runtime_monitor   # 读 /diagnostics，OK/WARN/ERROR 链路健康
rqt_reconfigure       # 运行时改限幅参数，立即影响当前指令
rqt_console           # 分级日志（超限告警已限流）
```

`rqt_plot` 的三条曲线是核心演示画面：原始输入含毛刺与超限，处理后输出平滑且
在限内，`flags` 位掩码标出每次干预的时刻与类型。

运行时调参示例（限幅在输出侧生效，改完立刻作用于当前指令）：

```bash
ros2 param set /velocity_guard max_linear_x 0.4
ros2 param list /velocity_guard
ros2 param describe /velocity_guard max_lateral_accel
```

非法参数会被拒绝并给出原因，节点保留原有配置继续运行：

```bash
$ ros2 param set /velocity_guard cmd_timeout 0.01
Setting parameter failed: cmd_timeout must span at least two control cycles, \
otherwise the watchdog trips on normal jitter
```

## 7. 本机环境注意事项（WSL2）

本机为多网卡环境（VPN 风格的 `eth0` 26.85.197.31/8 加上 `eth2`），**默认的
FastDDS 在此无法完成发现** —— 连 ROS 自带的 `talker`/`listener` 在两个独立进程
间也收不到消息。CycloneDDS 正常：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

与本项目代码无关，但在本机跑多进程时需要。另外 `ros2` CLI 守护进程会缓存过期
的图信息，切换 `ROS_DOMAIN_ID` 后 `ros2 node/topic list` 可能报告错误结果，先
`ros2 daemon stop`。

## 8. 目录结构

```
cmd_vel_safety_ws/
├── PROJECT_OUTLINE.md          设计文档
├── README.md                   本文件
├── docs/scenario.md            bag 场景标注 + 实测结果
└── src/
    ├── cmd_vel_safety_msgs/    接口包
    │   └── msg/{SafetyReport,MotionState}.msg
    └── cmd_vel_safety/
        ├── include/cmd_vel_safety/safety_limiter.hpp
        ├── src/safety_limiter.cpp          纯算法，不依赖 rclcpp
        ├── src/velocity_guard_node.cpp
        ├── src/motion_state_monitor_node.cpp
        ├── src/virtual_robot_node.cpp
        ├── launch/{bringup,replay_bag,record_bag,demo}.launch.py
        ├── config/params.yaml
        ├── scripts/check_bag.py            离线不变量校验
        ├── test/test_safety_limiter.cpp    19 项单元测试
        └── bags/cmd_vel/                   给定的测试数据
```

安全算法与 ROS 解耦是有意的：`safety_limiter` 编译为不含 `rclcpp` 依赖的静态库，
因此流水线逻辑可以用 gtest 做确定性测试（给定输入序列与 dt，断言输出），不受
ROS 图与调度抖动影响。
