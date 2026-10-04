# 速度安全与监控修正 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking. 用户确认前不执行实现；当前建议由主代理逐项执行。

**Goal:** 修复已复现的输出约束、部署连接和验证缺陷，并使参数与状态监控行为可明确验证。

**Architecture:** 保留三个 ROS 2 节点和独立 C++ 安全算法库。算法层保证最终输出约束，节点层负责参数、通信和反馈有效性，离线工具区分数值验证与录制完整性验证。分四个可独立验收的批次实施：算法与参数、连接与工具、监控、验证与文档。

**Tech Stack:** ROS 2 Humble、C++17、ament_cmake、gtest、Python 3、rosbag2、launch_testing。

**Spec:** 本轮用户需求及本文件“拟确认的行为决策”；现有架构参考 `../../../PROJECT_OUTLINE.md`。冲突之处以用户确认后的本方案为准。

## Global Constraints

- 必须订阅 `/cmd_vel`，通过 ROS Topic 发布处理结果与机器人运动状态。
- 默认三节点、现有 Topic 名称、现有速度参数默认值保留。
- 急停保持 Reliable + TransientLocal；解除时清空旧目标，等待新速度指令。
- 非急停周期遵守速度、加减速与横向加速度约束；硬急停允许输出直接归零。
- 不改变现有 bag 和历史实测记录；新结果使用新路径记录并明确标注。
- 修复先有失败用例，再实现，再验证；源码修改在用户确认后开始。
- 不自动推送远端。每批验证后形成可检查的差异；提交/推送范围按用户后续授权执行。

## 拟确认的行为决策

1. **最终输出约束**：线速度、角速度的变化共同满足 `abs(v*w) <= max_lateral_accel`。先完成允许的减速，再分配增速空间；同时争用横向加速度预算时优先线速度，限制角速度增长。不能只在最后强裁剪而破坏减速度约束。
2. **收紧参数**：如果当前输出不在新的速度/横向加速度范围内，拒绝整次更新并返回原因。先减速或停车，再重设。放宽参数和当前输出已满足的收紧立即接受。这替代“新限速已生效但输出暂时超限”的模糊语义。
3. **反向运动**：一个周期内跨零时，先按减速度到零，再用剩余周期按加速度反向起步；不把整周期制动预算用于反向加速。
4. **监控来源**：新增 `MotionState` 来源枚举 `SOURCE_UNKNOWN=0`、`SOURCE_ODOMETRY=1`、`SOURCE_COMMAND=2` 和字段 `uint8 velocity_source`。有效里程计优先；反馈失效时仅使用新鲜安全指令估计；两者均失效时状态为未知，避免把缺数据解释为静止。新增状态常量追加为 `STATE_UNKNOWN=10`，不重编号旧常量。
5. **录制时间**：新录制使用 `ros2 bag record --use-sim-time`，便于独立核对输入断流。旧录制仍可做原有逐点检查；时间基准不一致时明确报告无法完成严格断流审计。
6. **范围控制**：本轮修正已有功能及其验收工具。状态迟滞、图形界面美化和 GitHub Actions 留待后续，不加入本轮。

## Review Focus

- 正负号组合与跨零反向：Task 1 检验每轴可达速度及耦合约束，避免零附近预算错误。
- 启动配置与动态更新不一致：Task 2 对相同配置执行相同合法性检查。
- 自定义 Topic 与无效路径：Task 3、4 验证真实连接及录制路径错误信息。
- NaN/Inf、非法四元数与同时失去反馈/指令：Task 5 检验统计恢复和未知来源。
- 合法硬急停、漏帧及短暂超时：Task 6 按实际模式审计，不让合法行为误报或不完整数据静默通过。

---

## Task 1：最终输出约束与跨零加减速

**Files:**
- Modify: `src/cmd_vel_safety/src/safety_limiter.cpp`
- Modify: `src/cmd_vel_safety/include/cmd_vel_safety/safety_limiter.hpp`
- Test: `src/cmd_vel_safety/test/test_safety_limiter.cpp`

**Interfaces:** 保留 `SafetyLimiter::submit(const Twist6 &, double)` 与 `update(double, double)`；新增库函数 `Velocity2D constrainCoupledStep(const Velocity2D & previous, const Velocity2D & candidate, double lateral_limit)`，供单元测试直接验证。

- [x] 新增失败测试 `KeepsLateralBoundDuringCrossedTransition`：稳定 `(1.0,1.2)` 到目标 `(0.8,1.5)`，`dt=0.05`，每周期 `abs(v*w)<=1.2+1e-9`，并最终到达目标。
- [x] 新增 `CoupledStepHandlesAllSignsAndConverges`：四种符号组合、同时增速、一增一减、零速度均满足约束并能收敛，不在边界永久卡住。
- [x] 新增 `ReversalUsesRemainingAccelerationTime`：跨零输出符合“制动到零+剩余时间反向加速”的预算。
- [x] 运行新增测试，确认现有实现失败，再修改实现。
- [x] 修正 `slew` 的跨零预算；从 previous 到 candidate 的每轴区间中选最靠近零的值为可行起点，先将线速度向 candidate 恢复，再将角速度向 candidate 恢复，分别受乘积约束限制。各轴始终位于原可达区间内，禁止越过目标。
- [x] 修正输出因耦合约束受限时的 flags/reasons，保留目标级限制说明。
- [x] 运行 `colcon build --symlink-install`，再运行 `./build/cmd_vel_safety/test_safety_limiter`。新增测试和既有测试均通过；不通过调大容差掩盖违例。

## Task 2：统一参数校验和明确动态更新语义

**Files:**
- Modify: `src/cmd_vel_safety/include/cmd_vel_safety/safety_limiter.hpp`
- Modify: `src/cmd_vel_safety/src/safety_limiter.cpp`
- Modify: `src/cmd_vel_safety/src/velocity_guard_node.cpp`
- Modify: `src/cmd_vel_safety/config/params.yaml`
- Test: `src/cmd_vel_safety/test/test_safety_limiter.cpp`
- Create/Test: `src/cmd_vel_safety/test/test_node_contracts.py`

**Interfaces:** 新增 `std::string validateLimits(const Limits &, double control_rate_hz)` 与 `std::string validateEnvelopeUpdate(const Limits &, const Velocity2D & current)`；空字符串表示通过，否则为拒绝原因。

- [x] 增加边界测试：非有限参数、非法范围、20 Hz 下 `cmd_timeout=0.01` 必须拒绝；输出 `1.0` 时收紧为 `0.4` 必须拒绝，输出降至 `0.3` 后允许收紧为 `0.4`。
- [x] 增加启动/运行时集成用例：同一非法组合通过 YAML 或参数服务均被拒绝，运行时拒绝保留原参数与算法行为。
- [x] 确认失败后提取共享校验；启动阶段先验证再创建定时器，参数回调验证完整候选配置后再应用。
- [x] 对 QoS 枚举拒绝拼写错误；加载启动 `log_throttle_sec`，避免始终使用成员默认值。
- [x] `publishReport` 使用本周期开始采样的同一个 `rclcpp::Time` 作为 header，不再另取一次 now；算法 dt 与相邻报告时间差一致，异常 dt 的回退/上限行为由测试明确覆盖。
- [x] 运行算法测试和 `test_node_contracts.py` 中参数用例，检查失败原因、参数值和实际输出一致。

## Task 3：全链路 Topic 重映射

**Files:**
- Modify: `src/cmd_vel_safety/launch/bringup.launch.py`
- Test: `src/cmd_vel_safety/test/test_node_contracts.py`
- Modify: `src/cmd_vel_safety/CMakeLists.txt`
- Modify: `src/cmd_vel_safety/package.xml`

**Interfaces:** 沿用现有 launch 参数 `cmd_vel_topic` 与 `cmd_vel_safe_topic`，三个节点共享对应连接关系。

- [x] 增加集成测试：输入改为 `/test/raw`、输出改为 `/test/safe`，发布指令后网关有输出、虚拟底盘有运动反馈、监控有正常输入统计。
- [x] 确认现有 launch 在自定义 Topic 场景失败。
- [x] 给监控节点添加原始/安全 Topic remapping，给虚拟底盘添加安全 Topic remapping。
- [x] 注册 launch_testing 测试及直接测试依赖；测试使用独立 ROS_DOMAIN_ID，避免影响用户其他节点。
- [x] 验证默认 Topic 与自定义 Topic 两组用例均通过。

## Task 4：校验工具安装与录制路径

**Files:**
- Modify: `src/cmd_vel_safety/scripts/check_bag.py`
- Modify: `src/cmd_vel_safety/launch/record_bag.launch.py`
- Modify: `src/cmd_vel_safety/package.xml`
- Modify: `src/cmd_vel_safety/CMakeLists.txt`
- Create/Test: `src/cmd_vel_safety/test/test_check_bag.py`

**Interfaces:** 保留 `check_bag.py BAG --params FILE`；默认配置使用 `ament_index_python.packages.get_package_share_directory('cmd_vel_safety')` 定位。

- [x] 保存当前失败证据：symlink 安装下 `ros2 run cmd_vel_safety check_bag.py --help` 返回 No executable found。
- [x] 将脚本可执行位纳入 Git；为 `rclpy`、`rosbag2_py`、`rosidl_runtime_py`、`ament_index_python` 补齐直接运行依赖。
- [x] 替换默认配置定位方式；显式 `--params` 继续有效。
- [x] 录制输出使用展开用户目录后的绝对路径；相对路径 `output:=run1` 应正常工作，已有输出目录在启动子进程前明确拒绝。
- [x] 增加上述路径用例；新录制增加 `--use-sim-time`。
- [x] 分别用临时 build/install 目录验证普通安装与 symlink 安装；任意工作目录下 `ros2 run ... --help` 和默认配置读取均成功。

## Task 5：监控输入校验、来源和动态频率

**Files:**
- Modify: `src/cmd_vel_safety/src/motion_state_monitor_node.cpp`
- Modify: `src/cmd_vel_safety_msgs/msg/MotionState.msg`
- Test: `src/cmd_vel_safety/test/test_node_contracts.py`

**Interfaces:** `MotionState.velocity_source` 与 `STATE_UNKNOWN` 按行为决策定义；现有 state、health 等字段继续使用。

- [x] 加入失败用例：速度含 NaN/Inf、四元数零范数/非有限值时不更新有效反馈、不污染累计里程，正常反馈恢复后继续累计。
- [x] 加入来源用例：有效 odom -> ODOMETRY；odom 过期但安全指令新鲜 -> COMMAND 并提示缺反馈；两者过期 -> UNKNOWN，状态不可显示为确认静止。
- [x] 对新鲜安全指令使用 `safe_rate_.since(now)`；超时阈值沿用 `signal_lost_timeout`，不增加重复参数。
- [x] 有限且非零范数的四元数归一化后解算；异常消息不刷新有效 odom 的新鲜度，并报告诊断。
- [x] 校验监控参数的有限性、正频率和正统计窗口；更新 `publish_rate_hz` 后重建定时器。增加 5 Hz -> 10 Hz 的频率观测测试，允许正常调度误差。
- [x] 验证改频与异常恢复不会重置累计正常里程；重新构建消息包和所有消费节点。

## Task 6：离线审计与新录制验收

**Files:**
- Modify: `src/cmd_vel_safety/scripts/check_bag.py`
- Test: `src/cmd_vel_safety/test/test_check_bag.py`
- Modify: `src/cmd_vel_safety/CMakeLists.txt`

**Interfaces:** 新增 CLI `--strict` 与可选 `--reference-bag PATH`。`--strict` 要求审计所需 Topic、统一时间基准和可验证周期；`--reference-bag` 比较原始输入消息数量和顺序内容，NaN 按相同位置比较。保留历史 bag 的非严格逐点检查，显式列出未检查项。

- [x] 失败用例：普通 `dv/dt=2.0` 在上限 `0.8` 时必须失败；合法硬急停从 `1.0` 到 `0.0` 必须通过且验证输出为零。
- [x] 按普通加速、普通制动、紧急制动和跨零阶段分别核对预算；硬急停仅豁免斜率限制，仍检查有限性与零输出。不得以增加 5% 容差作为修复。
- [x] 严格模式对新录制的 raw 接收时间和报告 header 时间进行一致性检查；两者不一致时失败并说明原因，不跨钟计算。
- [x] 独立使用 `/cmd_vel` 接收间隔验证超时判定，边界允许一个控制周期及明确的小量记录延迟；不依赖 watchdog_active 决定是否存在断流。
- [x] 短超时后输入恢复允许正常继续运动，不要求在不足以完成制动的间隙内已经停车；足够长的断流必须在预算时间内停车。
- [x] 报告与安全输出按顺序检查内容和计数，检测缺少报告、缺少输出、非有限值和时间回跳；回跳切分为独立时间段，间断数据不作为连续性通过证据。
- [x] `--reference-bag` 对 320/310 帧场景必须失败。新录制采用新的输出路径，运行严格校验；若发现录制开头漏帧，再依据启动日志修正录制/回放就绪顺序，不靠无限加长固定等待证明完整。
- [x] 运行 `colcon test --packages-select cmd_vel_safety` 与 `colcon test-result --verbose`，全部通过；新录制必须证明 320 帧输入完整、相关安全输出/报告一致。

## Task 7：同步说明与最终验收

**Files:**
- Modify: `README.md`
- Modify: `PROJECT_OUTLINE.md`
- Modify: `docs/scenario.md`

- [x] 说明动态收紧拒绝策略、耦合输出优先级、监控来源和新校验命令。
- [x] 历史实测记录标注录制覆盖范围，新测试结果另列，不把先前的 310 帧录制改写为完整录制。
- [x] 整体检查：构建成功；旧测试与新增回归全部通过；默认/自定义 Topic 有数据；安装后的 checker 可运行；新 bag 严格审计通过。
- [x] 执行 `git diff --check` 并检查修改范围，交付修改摘要、验证证据和剩余限制。

## 用户确认点

- 是否接受动态收紧参数时拒绝不容纳当前输出的更新？推荐接受。
- 是否接受 `MotionState` 追加来源字段和 UNKNOWN 状态？需要重新构建使用该自定义消息的程序。
- 是否按上述完整范围执行？建议主代理在当前会话逐项实施；也可由用户明确选择分任务代理实施。
