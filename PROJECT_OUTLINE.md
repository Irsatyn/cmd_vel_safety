# 机器人速度指令处理与状态监控系统 — 设计大纲

ROS 2 Humble / C++17 / ament_cmake

---

## 1. 设计目标

在机器人软件栈中，`/cmd_vel` 是「上游决策」（导航、遥控、视觉跟随）与「下游执行」（底盘驱动）之间唯一的窄接口。上游节点可能崩溃、可能发散、可能输出物理上不可执行的指令，而底盘会忠实地执行它收到的任何东西。因此在这条链路上插入一个**速度安全网关**是实际工程中的标准做法（如 Nav2 的 `velocity_smoother`）。

本系统实现这样一个网关，并配套一个独立的状态监控节点，目标为：

| 目标 | 说明 |
|---|---|
| G1 安全 | 任何上游输入（包括 NaN/Inf/超限/突变）都不能让底盘收到危险指令 |
| G2 可用 | 正常指令必须以可忽略的延迟与失真通过，不能「为了安全而不可用」 |
| G3 失效安全 | 上游中断、挂死、数值崩坏时，机器人必须**主动可控地停车**，而不是保持最后速度 |
| G4 可观测 | 每一次安全干预都要可追溯：干预了什么、为什么、干预了多少 |
| G5 可调 | 全部阈值通过 ROS Parameter 暴露，支持运行时动态修改与 YAML 配置 |

---

## 2. 输入数据分析（基于 `cmd_vel.zip`）

设计前先解码了给定的 rosbag（`geometry_msgs/msg/Twist`，320 帧，35.9 s，10 Hz），它是一组**刻意构造的测试场景**，直接定义了系统必须具备的能力：

| 帧号 | 时间 (s) | 数据特征 | 暴露的需求 |
|---|---|---|---|
| 0–29 | 0.0–2.9 | 全零 | 静止状态识别 |
| 30–80 | 3.0–8.0 | `lx=0.3` | 正常匀速直行（不得失真） |
| 81–119 | 8.1–11.9 | `lx` 线性斜坡 0.3→0.8 | 平滑加速（不得被误判为异常） |
| 120–159 | 12.0–15.9 | `lx=0.5, az=0.6` | 弧线运动；入口处 `lx` 0.8→0.5、`az` 0→0.6 为阶跃 |
| 160–189 | 16.0–18.9 | `lx=-0.35` | 倒车（需独立的、更严格的反向限幅） |
| 190–219 | 19.0–21.9 | `lx=2.5, az=3.0` | **持续超限** → 需幅值限幅 + 加速度限幅 |
| 220 | 22.0 | `lx=NaN` | **非有限数值** → 需数值合法性校验 |
| 225 | 22.5 | `az=+Inf` | **非有限数值** |
| 230 | 23.0 | `lx=-2.2`（仅 1 帧） | **孤立毛刺** → 需脉冲抑制（且不能把 190 的持续超限也当毛刺丢掉） |
| 240–279 | 24.0–27.9 | `lx=0.3, ly=0.4` | **违反差速底盘非完整约束** → 需运动学投影 |
| — | 27.9→32.0 | **4.1 s 数据空洞** | **指令中断** → 需看门狗 + 受控制动 |
| 280–319 | 32.0–35.9 | `lx=0.2` → 0 | 中断恢复后不得「跳回」旧速度 |

**关键设计约束（来自第 190 帧与第 230 帧的对比）**：持续的大幅值指令（190–219）应当被**限幅后放行**，而单帧的大幅值指令（230）应当被**直接丢弃**。二者的瞬时跳变幅度相近，仅靠「跳变幅度」无法区分 —— 必须引入**时间维度上的确认机制**。这是本设计中脉冲抑制采用「候选—确认」状态机而非中值滤波的原因（详见 §6.3）。

---

## 3. 系统架构

三个职责单一的节点，通过 Topic 组成单向数据流：

```
  ┌──────────────┐
  │ rosbag play  │
  │   / teleop   │
  └──────┬───────┘
         │ /cmd_vel                 /e_stop
         │ geometry_msgs/Twist      std_msgs/Bool
         ▼                             │
  ┌───────────────────────────┐ ◄──────┘
  │   ① velocity_guard        │   安全处理（本系统核心）
  │   固定 20 Hz 输出          │
  └───┬───────────────────┬───┘
      │ /cmd_vel_safe     │ /velocity_guard/report
      │ Twist             │ SafetyReport
      ▼                   │
  ┌──────────────────┐    │
  │ ② virtual_robot  │    │   被控对象模型（含一阶电机滞后）
  └────┬─────────────┘    │
       │ /odom            │
       │ nav_msgs/Odometry│
       ▼                  ▼
  ┌────────────────────────────────────────┐
  │        ③ motion_state_monitor          │   状态与健康监控
  └──┬──────────────┬──────────────────┬───┘
     │              │                  │
/robot_motion_state │ /robot_status    │ /diagnostics
   MotionState      │ std_msgs/String  │ DiagnosticArray
                    │                  └─► rqt_runtime_monitor
                    └─► 人可读单行摘要
```

`motion_state_monitor` 同时订阅原始 `/cmd_vel`，用于独立统计上游输入频率并与 `velocity_guard` 的判断交叉验证 —— 监控节点不信任被监控节点的自述。

### 为什么这样划分

- **`velocity_guard`（安全）** 处在实时控制路径上，必须轻量、无阻塞、无动态内存分配热点。它只做一件事：把不可信的 Twist 变成可信的 Twist。
- **`motion_state_monitor`（观测）** 不在控制路径上，可以做窗口统计、状态机、字符串拼装等相对「重」的工作。即使它崩溃，机器人仍然安全 —— 这是把它独立出来的核心理由。
- **`virtual_robot`（被控对象）** 让数据流闭环，在没有真实底盘时也能验证「限幅后的指令是否真的产生了平滑运动」，并为监控节点提供「实际速度 vs 指令速度」的跟踪误差输入。

---

## 4. Topic 设计

| Topic | 类型 | 方向 | QoS | 说明 |
|---|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | guard ← 上游 | BestEffort, depth 10 | 原始指令，不可信 |
| `/e_stop` | `std_msgs/Bool` | guard ← 上游 | **Reliable, TransientLocal**, depth 1 | 急停；latched，后启动的 guard 也能立即收到当前急停状态 |
| `/cmd_vel_safe` | `geometry_msgs/Twist` | guard → 底盘 | Reliable, depth 10 | 处理后指令，固定 20 Hz |
| `/velocity_guard/report` | `cmd_vel_safety_msgs/SafetyReport` | guard → monitor | Reliable, depth 10 | 逐周期干预明细 |
| `/odom` | `nav_msgs/Odometry` | robot → monitor | Reliable, depth 20 | 实际运动反馈 |
| `/robot_motion_state` | `cmd_vel_safety_msgs/MotionState` | monitor → | Reliable, depth 10 | 结构化状态，供 rqt_plot |
| `/robot_status` | `std_msgs/String` | monitor → | Reliable, depth 10 | 人可读摘要，供 `ros2 topic echo` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | monitor → | Reliable, depth 10 | 标准诊断，供 rqt_runtime_monitor |

**QoS 选择说明**：`/cmd_vel` 订阅端用 **BestEffort**，因为 BestEffort 订阅者兼容 Reliable 与 BestEffort 两种发布者，而 Reliable 订阅者收不到 BestEffort 发布者的数据。这保证了无论上游是 `rosbag play`（默认 Reliable）、`teleop_twist_keyboard`（Reliable）还是某些 BestEffort 的实时控制器，网关都能工作。该项仍由参数 `cmd_vel_qos_reliability` 可切换。

---

## 5. 自定义消息设计

放在独立的接口包 `cmd_vel_safety_msgs` 中（ROS 2 推荐做法：接口与实现解耦，下游可以只依赖消息包）。

### 5.1 `SafetyReport.msg`

```
std_msgs/Header header

# 干预标志位（位掩码，可同时置位）
uint16 FLAG_NONE                 = 0
uint16 FLAG_NON_FINITE           = 1      # 输入含 NaN/Inf，整帧丢弃
uint16 FLAG_NONHOLONOMIC         = 2      # ly/lz/ax/ay 非零，已投影归零
uint16 FLAG_SPIKE_REJECTED       = 4      # 孤立跳变，待确认，本周期沿用旧值
uint16 FLAG_LINEAR_CLAMPED       = 8      # 线速度幅值限幅
uint16 FLAG_ANGULAR_CLAMPED      = 16     # 角速度幅值限幅
uint16 FLAG_LATERAL_ACCEL_LIMITED= 32     # v·ω 向心加速度耦合限幅
uint16 FLAG_ACCEL_LIMITED        = 64     # 加/减速度斜率限幅
uint16 FLAG_DEADBAND_APPLIED     = 128    # 死区归零
uint16 FLAG_WATCHDOG_TIMEOUT     = 256    # 输入超时，受控制动
uint16 FLAG_ESTOP                = 512    # 急停生效
uint16 FLAG_SAFETY_HOLD          = 1024   # 连续非法输入超阈值，安全保持

uint16 flags
string[] reasons                 # 人可读原因，与 flags 对应

geometry_msgs/Twist input_cmd    # 本周期参考的最新原始输入
geometry_msgs/Twist target_cmd   # 校验/限幅后的目标
geometry_msgs/Twist output_cmd   # 实际发布到 /cmd_vel_safe 的指令

float64 input_rate_hz            # guard 侧测得的上游频率
float64 time_since_last_input    # s
bool watchdog_active
bool estop_active

uint64 total_received            # 累计统计
uint64 total_rejected            # 整帧丢弃数（NaN/Inf/毛刺）
uint64 total_modified            # 被改写数
uint64 total_published
```

**设计取舍**：同时提供位掩码 `flags` 与字符串数组 `reasons`。位掩码便于 `rqt_plot` 画时间曲线、便于下游程序判断；字符串便于人在 `ros2 topic echo` 里直接读懂。二者都带上 `input/target/output` 三元组，使「输入什么 → 想要什么 → 实际给了什么」在单条消息内自证。

### 5.2 `MotionState.msg`

```
std_msgs/Header header

uint8 STATE_IDLE           = 0
uint8 STATE_FORWARD        = 1
uint8 STATE_REVERSE        = 2
uint8 STATE_TURN_IN_PLACE  = 3
uint8 STATE_ARC_LEFT       = 4
uint8 STATE_ARC_RIGHT      = 5
uint8 STATE_BRAKING        = 6
uint8 STATE_SAFETY_HOLD    = 7    # 看门狗/连续非法输入导致的保持
uint8 STATE_EMERGENCY_STOP = 8    # 急停latched
uint8 STATE_SIGNAL_LOST    = 9    # 完全无上游输入

uint8 STATE_UNKNOWN = 10
uint8 SOURCE_UNKNOWN = 0
uint8 SOURCE_ODOMETRY = 1
uint8 SOURCE_COMMAND = 2
uint8 velocity_source
uint8 state
string state_name

uint8 HEALTH_OK    = 0
uint8 HEALTH_WARN  = 1
uint8 HEALTH_ERROR = 2
uint8 HEALTH_STALE = 3
uint8 health
string health_message

# 运动学量（取自 /odom 实测）
float64 linear_speed             # m/s，带符号
float64 angular_speed            # rad/s
float64 turn_radius              # m；直行时为 inf
float64 heading                  # rad
float64 distance_travelled       # m，累计路程
float64 duration_in_state        # s

# 链路健康
float64 cmd_input_rate_hz        # monitor 独立测得的 /cmd_vel 频率
float64 safe_cmd_rate_hz
float64 time_since_last_cmd
float64 tracking_error_linear    # |指令 - 实测|
float64 tracking_error_angular
bool input_frozen                # 上游长时间重复同一数值（疑似挂死）

uint32 intervention_count        # 累计安全干预次数
uint16 last_safety_flags
```

---

## 6. 安全处理流水线（`velocity_guard` 核心）

节点采用**固定速率（默认 20 Hz）定时器驱动输出**，而非「收到一帧就发一帧」。这是一个关键设计决策：

1. 加速度限幅需要确定的 `dt`，输入频率抖动会直接污染斜率计算；
2. 看门狗必须在**没有输入**时仍然能发出制动指令 —— 事件驱动模型做不到这一点；
3. 底盘驱动通常期望稳定的指令流，输出频率与输入频率解耦后，上游抖动不会传导到执行层。

处理分两个阶段。`submit()` 在每帧输入到达时执行校验，`update()` 在每个定时器周期执行限幅与输出。

```
 ┌─ submit(raw, t)  每帧输入触发 ────────────────────────────────┐
 │                                                               │
 │  S1 有限性校验     6 个分量全部 isfinite？否 → 整帧丢弃        │
 │       │                              consecutive_invalid++    │
 │       │                              ≥N → SAFETY_HOLD         │
 │       ▼                                                       │
 │  S2 非完整约束投影  ly/lz/ax/ay 置零（差速底盘不可执行）       │
 │       ▼                                                       │
 │  S3 脉冲抑制       跳变 > 阈值 → 列为候选，沿用旧目标；        │
 │       │             下一帧确认则采纳，不确认则丢弃             │
 │       ▼                                                       │
 │  S4 死区           |v|<dz → 0（避免电机堵转嗡鸣）              │
 │       ▼                                                       │
 │      target_  （已接受的目标速度，未限幅）                     │
 └───────────────────────────────────────────────────────────────┘
 ┌─ update(t, dt)  固定 20 Hz ───────────────────────────────────┐
 │                                                               │
 │  U0 有效目标       急停 / 看门狗 / 安全保持 → 目标强制为 0     │
 │       ▼                                                       │
 │  U1 幅值限幅       v ∈ [min_linear_x, max_linear_x]（反向更严）│
 │       │             ω ∈ [-max_angular_z, +max_angular_z]       │
 │       ▼                                                       │
 │  U2 向心加速度耦合  |v·ω| ≤ a_lat_max → 按 |v| 压缩 ω          │
 │       ▼                                                       │
 │  U3 斜率限幅       加速用 accel 限、减速用 decel 限（通常更大）│
 │       │             急停/超时时乘 emergency_decel_factor       │
 │       ▼                                                       │
 │  U4 急停硬停       estop_hard_stop=true → 直接输出 0           │
 │       ▼                                                       │
 │      /cmd_vel_safe  +  /velocity_guard/report                 │
 └───────────────────────────────────────────────────────────────┘
```

**为什么限幅放在 `update()` 而不是 `submit()`**：已接受的参数在下一输出周期作用于当前目标，不必等待新输入。动态收紧时先检查当前输出能否满足新速度和横向加速度范围；不能满足则拒绝整次更新，提示先减速或停车。这样已生效的范围始终与实际输出约束一致。

### 6.1 有限性校验（对应第 220、225 帧）

NaN/Inf 出现在任一分量时**整帧丢弃**，而不是只把坏分量置零。理由：NaN 说明上游的计算已经崩坏（通常是除零或未初始化内存），同一帧里其余分量的可信度同样为零。丢弃后沿用上一个合法目标，并计数；连续非法达到 `max_consecutive_invalid`（默认 5）时判定上游持续故障，进入 `SAFETY_HOLD` 受控停车。

这给出了「容忍偶发坏帧」与「不容忍持续故障」之间的正确行为：bag 中的两处非法值是孤立的，系统平滑跨过；若上游真的发散，0.5 s 内即停车。

### 6.2 非完整约束投影（对应第 240–279 帧）

差速/阿克曼底盘只能执行 `linear.x` 与 `angular.z`。`linear.y=0.4` 这种指令若被忽略，机器人表现出的运动将与上游的意图不符（上游以为自己在斜向平移），属于隐蔽的行为偏差。系统显式将 `ly/lz/ax/ay` 归零并上报 `FLAG_NONHOLONOMIC`，把隐藏的不一致变成可见的告警。全向底盘可通过参数 `enforce_nonholonomic=false` 关闭。

### 6.3 脉冲抑制：候选—确认状态机（对应第 230 帧 vs 190 帧）

```
          |Δv| > thr 或 |Δω| > thr ?
                   │
         ┌─── 否 ──┴── 是 ───┐
         ▼                   ▼
   直接采纳为目标      与现有候选一致？
   清除候选            ┌── 是 ──┴── 否 ──┐
                       ▼                 ▼
                 confirm_count++     设为新候选
                 ≥ N → 采纳          count=0
                                     沿用旧目标
                                     FLAG_SPIKE_REJECTED
```

- 第 230 帧 `-2.2`：成为候选，输出沿用 `0.25`；第 231 帧回到 `0.25`（与目标一致）→ 走「直接采纳」分支，候选被清除，毛刺**从未进入输出**。
- 第 190 帧 `2.5`：成为候选；第 191 帧同为 `2.5` → 确认次数达标 → 采纳，随后由 U1 限幅到 `max_linear_x`。

代价是真实阶跃指令会有 1 个输入周期（≈100 ms）的额外延迟。考虑到 U3 的斜率限幅本身就会把阶跃拉成斜坡，这个延迟在实际响应中不可感知。相比中值滤波的优点是：不对**所有**指令引入延迟，只对可疑指令引入。

### 6.4 向心加速度耦合限幅

单独限制 `v` 和 `ω` 不足以保证安全：`v=1.0, ω=1.5` 两者都在限内，但向心加速度 `v·ω = 1.5 m/s²` 足以让高重心机器人侧翻或让载物滑落。系统约束 `|v·ω| ≤ max_lateral_accel`，超出时按 `ω ← sign(ω)·a_max/|v|` 压缩角速度（保留平动意图，牺牲转向半径）。

最终输出也必须满足耦合约束。各轴先按加减速度计算本周期可达区间，再选区间内最靠近零的速度作为起点，依次恢复允许的线速度和角速度进度。该顺序先利用减速释放预算，再约束增速，避免目标合法但过渡输出超限。反向跨零时分别计算制动到零和反向加速所用时间。

### 6.5 看门狗（对应 27.9→32.0 s 空洞）

超过 `cmd_timeout`（默认 0.5 s）未收到输入即判定链路中断：

1. 将 `target_` **重置为零**（而非仅把输出拉零）；
2. 以 `decel × emergency_decel_factor` 的斜率受控减速到 0；
3. 持续发布零指令并置 `FLAG_WATCHDOG_TIMEOUT`。

重置 `target_` 而非仅压制输出，是为了保证**恢复行为安全**：32.0 s 输入恢复时，新指令 `0.2` 是相对「静止」而非相对「中断前的 0.3」被评估的，机器人从 0 平滑加速到 0.2，不会出现「跳回旧速度」的危险动作。若恢复时的首帧指令很大，它还必须经过 §6.3 的确认流程才会被采纳。

选择「受控减速」而不是「立即输出 0」，是因为后者让底盘以无穷大减速度制动，实际会造成机械冲击、打滑和里程计跳变。急停（`/e_stop`）是唯一允许硬停的路径。

### 6.6 急停

`/e_stop` 用 **TransientLocal** QoS，使 guard 节点在急停已经置位之后才启动时，依然能立刻收到 `true` 并保持停止 —— 避免「重启节点 = 解除急停」这一典型事故模式。急停为 latched 状态，必须显式发布 `false` 才解除。

**这个选择有一个必须明确记录的代价**：Volatile 发布者与 TransientLocal 订阅者在 DDS 下**不兼容，且表现为静默地什么都不投递**。裸 `ros2 topic pub` 与不带 QoS 元数据的 `ros2 bag play` 默认都是 Volatile，因此操作者必须显式指定：

```bash
ros2 topic pub /e_stop std_msgs/msg/Bool '{data: true}' \
  --qos-durability transient_local --qos-reliability reliable
```

权衡结论是**安全优先保留 TransientLocal 作为默认**，但提供参数 `estop_qos_durability` 可切换为 `volatile` 以兼容任意发布者（代价是失去「启动即保持急停」的能力），并在 README 中显式给出正确的发布命令。把这个陷阱写进文档而不是悄悄放宽 QoS，是因为「急停发不进去且没有任何报错」比「急停在重启后失效」更危险。

---

## 7. 参数设计

速度网关的数值参数通过 `ParameterDescriptor` 声明范围，启动配置与运行时更新共用有限性、范围和跨字段校验，并注册 `on_set_parameters_callback` 做**合法性校验**（如 `max_linear_x > 0`、`min_linear_x ≤ 0`、`control_rate_hz > 0`），非法设置被拒绝并返回原因，节点保持上一组有效参数继续运行。参数本身也是一个需要防御的外部输入。

### `velocity_guard`

| 参数 | 默认 | 单位 | 说明 |
|---|---|---|---|
| `control_rate_hz` | 20.0 | Hz | 固定输出频率 |
| `cmd_vel_qos_reliability` | `best_effort` | — | 输入 QoS |
| `max_linear_x` | 1.0 | m/s | 前进上限 |
| `min_linear_x` | -0.3 | m/s | 倒车下限（更严，后向无感知） |
| `max_angular_z` | 1.5 | rad/s | 角速度上限 |
| `max_linear_accel` | 0.8 | m/s² | 线加速 |
| `max_linear_decel` | 1.5 | m/s² | 线减速（大于加速：制动应更快） |
| `max_angular_accel` | 2.0 | rad/s² | 角加速 |
| `max_angular_decel` | 3.0 | rad/s² | 角减速 |
| `max_lateral_accel` | 1.2 | m/s² | 向心加速度耦合上限 |
| `enforce_nonholonomic` | true | — | 非完整约束投影 |
| `max_consecutive_invalid` | 5 | 帧 | 连续非法输入容忍上限 |
| `spike_rejection_enabled` | true | — | 脉冲抑制开关 |
| `spike_linear_threshold` | 0.6 | m/s | 需确认的线速度跳变 |
| `spike_angular_threshold` | 1.2 | rad/s | 需确认的角速度跳变 |
| `spike_confirm_count` | 1 | 帧 | 确认所需额外帧数 |
| `linear_deadband` | 0.01 | m/s | 线速度死区 |
| `angular_deadband` | 0.02 | rad/s | 角速度死区 |
| `cmd_timeout` | 0.5 | s | 看门狗超时 |
| `emergency_decel_factor` | 2.0 | — | 紧急制动减速度倍数 |
| `estop_hard_stop` | true | — | 急停是否绕过斜率限幅 |
| `estop_qos_durability` | `transient_local` | — | 急停 QoS，见 §6.6 的兼容性取舍 |

### `motion_state_monitor`

| 参数 | 默认 | 说明 |
|---|---|---|
| `publish_rate_hz` | 5.0 | 状态发布频率 |
| `idle_linear_threshold` | 0.02 | m/s，静止判定 |
| `idle_angular_threshold` | 0.05 | rad/s，静止判定 |
| `turn_in_place_linear_threshold` | 0.05 | m/s，原地转向判定 |
| `expected_input_rate_hz` | 10.0 | 期望上游频率（偏离即告警） |
| `input_rate_tolerance` | 0.5 | 允许的相对偏差 |
| `rate_window` | 2.0 | s，频率估计滑动窗口 |
| `signal_lost_timeout` | 1.0 | s，判定 SIGNAL_LOST |
| `frozen_input_timeout` | 10.0 | s，上游数值冻结告警 |
| `tracking_error_warn` | 0.15 | m/s，跟踪误差告警 |

### `virtual_robot`

| 参数 | 默认 | 说明 |
|---|---|---|
| `update_rate_hz` | 50.0 | 积分频率 |
| `linear_time_constant` | 0.15 | s，一阶电机滞后 |
| `angular_time_constant` | 0.10 | s |
| `cmd_timeout` | 0.5 | s，底盘侧独立看门狗（纵深防御） |
| `odom_frame` / `base_frame` | `odom` / `base_link` | TF 坐标系 |
| `publish_tf` | true | 是否广播 TF |

`virtual_robot` 保留**独立的看门狗**：即使上游的 `velocity_guard` 整个进程崩溃，底盘模型自己也会在 0.5 s 后停车。真实系统中这一层应由驱动固件实现。

---

## 8. 异常场景处理矩阵

| 异常 | 检测方式 | 系统响应 | 上报 |
|---|---|---|---|
| NaN / Inf | `std::isfinite` 全分量 | 整帧丢弃，沿用旧目标 | `FLAG_NON_FINITE` |
| 上游持续输出非法值 | 连续非法计数 ≥ 5 | 受控减速至 0 并保持 | `FLAG_SAFETY_HOLD` / `STATE_SAFETY_HOLD` |
| 速度超出机械极限 | 幅值比较 | 限幅后放行 | `FLAG_LINEAR/ANGULAR_CLAMPED` |
| 指令阶跃过大 | 斜率比较 | 按加速度上限平滑 | `FLAG_ACCEL_LIMITED` |
| 孤立数据毛刺 | 候选—确认状态机 | 丢弃，输出不受影响 | `FLAG_SPIKE_REJECTED` |
| 高速急转（倾覆风险） | `\|v·ω\|` 比较 | 压缩 ω | `FLAG_LATERAL_ACCEL_LIMITED` |
| 不可执行的自由度 | `ly/lz/ax/ay` 非零 | 投影归零 | `FLAG_NONHOLONOMIC` |
| 上游中断 / 节点崩溃 | 看门狗 0.5 s | 受控减速至 0；目标重置 | `FLAG_WATCHDOG_TIMEOUT` |
| 上游挂死（重复同值） | 数值冻结计时 | 仅告警（匀速巡航是合法的） | `MotionState.input_frozen` |
| 上游频率异常 | 滑动窗口 Hz 估计 | 告警 | `HEALTH_WARN` |
| 恢复后速度跳变 | 看门狗重置目标 | 从 0 重新加速 | — |
| 急停在节点启动前置位 | TransientLocal QoS | 启动即保持停止 | `FLAG_ESTOP` |
| 参数被设为非法值 | `on_set_parameters_callback` | 拒绝，保留原值 | 拒绝原因 |
| 下游节点崩溃 | `virtual_robot` 自带看门狗 | 底盘侧独立停车 | — |
| 时间回跳（bag 循环/仿真重置） | 节点时钟早于上一周期 | 重置安全状态和监控统计，重新计时 | 日志 |
| 异常控制周期 | 非正/非有限 `dt` 或正值大于 1 s | 前者使用上次有效周期；后者积分预算封顶 1 s | 算法回归测试 |

---

## 9. Launch 设计

| Launch 文件 | 用途 |
|---|---|
| `bringup.launch.py` | 启动三节点 + YAML 参数。参数：`params_file`、`use_sim_time`、`enable_virtual_robot`、`log_level`、各 topic 重映射 |
| `replay_bag.launch.py` | `bringup` + `ros2 bag play`（含 `--clock`、`rate`、`loop`），回放给定 bag 包含的异常场景 |
| `record_bag.launch.py` | `bringup` + `ros2 bag record` 全链路话题，产出可回放的验证数据 |
| `demo.launch.py` | `replay_bag` + `rqt_graph` + `rqt_plot`（预置曲线），一键演示 |

全部使用 `DeclareLaunchArgument` + `LaunchConfiguration`，不写死路径；bag 路径通过 `get_package_share_directory` 定位。

---

## 10. rosbag 使用

1. **输入**：`cmd_vel.zip` 解包后随包安装到 `share/cmd_vel_safety/bags/cmd_vel`，`replay_bag.launch.py` 直接回放，`--clock` + `use_sim_time:=true` 使三个节点使用 bag 时间，保证看门狗与斜率计算基于回放时间轴而非墙钟。
2. **输出**：`record_bag.launch.py` 录制 `/cmd_vel`、`/cmd_vel_safe`、`/velocity_guard/report`、`/odom`、`/robot_motion_state`、`/diagnostics`，用于离线核查每一次干预。
3. **验证脚本** `scripts/check_bag.py`：读取录制结果，自动断言 —— 输出中无 NaN/Inf、`|v| ≤ max_linear_x`、逐帧斜率 `≤ max_accel`、毛刺帧未出现在输出、空洞期间输出收敛到 0。把「人眼看曲线」变成可重复的回归测试。

## 11. rqt / rqt_graph 使用

- `rqt_graph`：验证三节点通过 8 个 topic 连成预期的单向数据流，无悬空订阅。
- `rqt_plot`：核心演示画面，三条曲线叠加
  `/cmd_vel/linear/x`（原始，含毛刺与超限）
  `/cmd_vel_safe/linear/x`（处理后，平滑且在限内）
  `/velocity_guard/report/flags`（干预位掩码，指出每处干预的时刻与类型）
- `rqt_runtime_monitor`：读取 `/diagnostics`，以 OK/WARN/ERROR 展示链路健康。
- `rqt_console`：观察分级日志（超限用 `THROTTLE` 限流，避免日志风暴）。
- `rqt_reconfigure`：运行时调整限幅参数，观察被接受的参数更新对 `/cmd_vel_safe` 的影响，收紧前需满足当前输出检查。

---

## 12. 目录结构

```
cmd_vel_safety_ws/
├── 设计大纲.md                     ← 本文件
├── README.md                       ← 构建与运行说明
├── docs/scenario.md                ← bag 场景逐段标注与预期结果
└── src/
    ├── cmd_vel_safety_msgs/        ← 接口包（与实现解耦）
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/{SafetyReport,MotionState}.msg
    └── cmd_vel_safety/             ← 实现包
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/cmd_vel_safety/safety_limiter.hpp
        ├── src/
        │   ├── safety_limiter.cpp              ← 纯算法，不依赖 rclcpp
        │   ├── velocity_guard_node.cpp
        │   ├── motion_state_monitor_node.cpp
        │   └── virtual_robot_node.cpp
        ├── launch/{bringup,replay_bag,record_bag,demo}.launch.py
        ├── config/params.yaml
        ├── scripts/check_bag.py
        ├── test/test_safety_limiter.cpp        ← gtest 单元测试
        └── bags/cmd_vel/                       ← 给定的测试数据
```

**安全算法与 ROS 解耦**：`safety_limiter.{hpp,cpp}` 编译为独立静态库，不包含任何 `rclcpp` 依赖。这样流水线逻辑可以用 gtest 做确定性单元测试（给定输入序列与 dt，断言输出），不需要启动 ROS 图、不受调度抖动影响。节点层只负责参数读取、QoS、定时器与消息装配。

---

## 13. 验证计划

| 层次 | 方法 | 判据 |
|---|---|---|
| 单元 | `test_safety_limiter.cpp`，逐条规则构造输入序列 | 每条规则的边界行为正确；毛刺/超限可区分 |
| 集成 | `replay_bag.launch.py` 回放给定 bag | §8 全部异常均被检出且响应正确 |
| 回归 | `check_bag.py` 对录制结果做断言 | 输出恒满足安全不变量 |
| 可视化 | `rqt_plot` / `rqt_graph` | 曲线与拓扑符合预期 |

**核心安全不变量**（在任何输入下都必须成立，由单元测试与 `check_bag.py` 双重检查）：

1. `/cmd_vel_safe` 的 6 个分量永远有限（无 NaN/Inf）；
2. `min_linear_x ≤ v ≤ max_linear_x` 且 `|ω| ≤ max_angular_z`；
3. 普通加速、普通制动与紧急制动分别满足配置预算；跨零分配两段时间，硬急停允许直接输出零；
4. `|v·ω| ≤ max_lateral_accel`；
5. 输入中断超过 `cmd_timeout` 后，输出在有限时间内收敛到 0 并保持；
6. 若 `enforce_nonholonomic`，输出的 `ly/lz/ax/ay` 恒为 0。

### 13.1 历史实测结果（修正前）

| 层次 | 结果 |
|---|---|
| `colcon test` | 50 项全通过（19 项 gtest + lint/格式检查），0 失败 |
| 回放给定 bag | §8 矩阵中 bag 所含的 8 条规则全部检出并正确响应 |
| 在线测试 | 死区 / 持续非法输入 / 急停 3 条规则全部通过 |
| `check_bag.py` | `PASSED: all safety invariants hold for this run`（718 条输出消息） |
| 计数器自证 | `received=320`（与 bag 消息数一致）、`rejected=6`、`modified=40`（恰为 `ly=0.4` 的 40 帧） |

逐段实测数值见 [docs/scenario.md](docs/scenario.md)。其中三项值得强调：

- 第 230 帧的单帧毛刺 `lx=-2.2` 在输出上**完全不可见**，而第 190 帧起持续的 `lx=2.5` 被正常采纳后限幅 —— §6.3 的候选—确认机制达到了设计目的。
- 第 190–219 段角速度最终为 **1.200** 而非限幅上限 1.500，是 `max_angular_z` 与向心加速度耦合限幅两级串联的结果；单独看任一参数都无法解释该数值。
- 中断恢复后输出从 **0.040 爬升到 0.200**，而非跳回中断前的 0.300，验证了 §6.5 中「重置 target 而非仅压制输出」的必要性。

### 13.2 校验方法上的一处修正

最初 `check_bag.py` 用 rosbag2 的接收时间戳计算 `dv/dt`，报出 `3.007 > 3.000 m/s²` 的违例。这是**校验脚本自身的缺陷**：`use_sim_time` 下节点以仿真时钟计算加速度预算，而 rosbag2 的接收时间戳取自系统时钟，跨时钟相除没有意义。`/cmd_vel_safe` 是无 header 的裸 `Twist`，无法提供节点侧时间戳；改为在 `/velocity_guard/report` 上校验后，`header.stamp` 与同一条消息里的 `output_cmd` 来自同一时钟，不变量 3 随即通过。

这也反过来说明了为什么 `SafetyReport` 要带 `header` 并同时携带 `input/target/output` 三元组：它让每一个输出都能在单条消息内被自证与离线复核。

## 14. 安全修正后的接口与验证

- `MotionState` 追加 `velocity_source` 与 `STATE_UNKNOWN=10`，旧状态编号不变。有效里程计优先，新鲜安全指令可作降级估计；两者失效时为 UNKNOWN。
- 非有限速度、非有限或零范数四元数不更新有效反馈，不污染累计里程。有限四元数归一化后用于朝向计算。
- `publish_rate_hz` 动态修改重建监控定时器；非法频率、窗口与阈值被拒绝。
- `SafetyReport.header.stamp` 与控制计算使用同一次时钟采样。
- 新录制使用仿真时间；严格审计独立检查原始输入间隔、控制报告连续性以及输出/报告一致性。参考 bag 比较可证明原始输入完整性。
- 本轮验证记录与旧数据的区别见 [场景文档第 7 节](docs/scenario.md#7-安全修正后的验证)。
