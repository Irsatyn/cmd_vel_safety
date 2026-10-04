# GitHub 交付证据 Implementation Plan

**Goal:** 将完整 ROS 2 项目、可复核运行证据、完整录屏和 AI 使用说明发布到用户指定仓库。

**Architecture:** 保留现有 C++ 节点。独立 Python 演示工具启动真实节点、按时间发布指令、采集实时状态与 rqt_graph，并录制其自身窗口；交付文档链接真实证据。

**Tech Stack:** ROS 2 Humble、rclpy、PyQt5、rqt_graph、OpenCV H.264、Git/GitHub。

**Spec:** 本轮用户的 GitHub 仓库交付与 README 清单。

## Global Constraints

- 不伪造截图、视频、运行输出或提交历史。
- 仅录制演示窗口，不采集用户桌面的其他窗口。
- 不修改已验收的 C++ 处理逻辑；演示使用独立 ROS Domain。
- 保留至少三条真实且有意义的提交；发布到 `Irsatyn/cmd_vel_safety`。
- README 明确 AI 使用及本机验证范围。

## Review Focus

- 发布者实际匹配急停 TransientLocal + Reliable QoS。
- 图像与视频来自运行中的节点数据与真实 Qt 窗口。
- 演示结束后清理本工具启动的节点进程。
- 视频覆盖完整过程，实际消息证据证明各场景生效。
- 仓库中媒体文件可访问，大小符合普通 Git 上传要求。

## Task 1：真实演示和证据

**Files:** `tools/capture_demo.py`、`docs/evidence/`。

- [x] 启动三个节点，显示 input/target/output、odom、状态与实际进程日志。
- [x] 以匹配的 QoS 发送急停，演示正常、超限、倒车、断流、非法输入、急停、解除后等待新指令及停车。
- [x] 录制完整窗口为 H.264 MP4，保存 rqt_graph 图像和运行图拓扑文本。
- [x] 用实际报告验证场景结果、有限性、速度和横向加速度边界；读取视频确认可解码、时长和尺寸。

## Task 2：README 交付说明

**Files:** `README.md`、`docs/evidence/README.md`。

- [x] README 覆盖结构、节点、Topic、关键参数、编译运行、功能验证和 AI 使用。
- [x] 提供录屏、截图、数据、复现命令和已有测试结果链接。
- [x] 明确屏幕采集方式与虚拟底盘，避免暗示真实硬件测试。

## Task 3：提交与上传

- [x] 验证媒体、文档链接和 Git 差异；保留现有三条有意义提交。
- [x] 提交交付证据和文档；推送 main 到已有 GitHub 仓库。
- [x] 通过 GitHub API 核实仓库、媒体和提交历史，交付仓库链接。
