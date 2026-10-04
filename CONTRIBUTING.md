# 开发与贡献

仓库根目录是 ROS 2 colcon 工作空间，环境要求及依赖安装见 [README](README.md)。

## 文件组织

- `src/cmd_vel_safety/`：C++ 算法、节点、头文件、Launch、参数、测试和演示 bag。
- `src/cmd_vel_safety_msgs/`：ROS 消息接口包。
- `docs/`：系统设计、场景说明、验收记录与运行证据；方案归入 `docs/plans/`。
- `tools/`：开发与证据采集辅助工具。
- 生成文件保留在 `build/`、`install/`、`log/`，由 `.gitignore` 排除。

## 验证修改

在仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
git diff --check
```

本机多网卡 / WSL2 环境需要时使用 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；实进程测试使用独立 ROS Domain。
算法变更应覆盖相应边界；消息定义变更需要重新构建消费节点；文档移动后需更新相对链接。

## 提交与问题反馈

每次提交描述一个可审阅的变更，附相关验证结果，保留真实历史。不要提交缓存、环境文件或凭证。
问题反馈请提供 ROS 版本、命令、参数和日志，入口为 [GitHub Issues](https://github.com/Irsatyn/cmd_vel_safety/issues)。
AI 辅助修改请如实说明用途，项目现有披露见 README 的 AI 使用说明。
