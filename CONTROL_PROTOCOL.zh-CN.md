# 推理录像本地控制接口 — v1

[English](CONTROL_PROTOCOL.md)

仅控制相机录像，不含 ROS、机器人指令、策略输入或物理同步保证。推理侧负责在自己的启动/退出流程接入；本包不启动推理。

## 启动与客户端

```bash
./build/dvp_monitor --auto --fps 60 --control-socket auto
/usr/bin/python3 tools/recording_control.py status
/usr/bin/python3 tools/recording_control.py start --run-id trial_20261110_001 --model checkpoint-A
/usr/bin/python3 tools/recording_control.py status --run-id trial_20261110_001
/usr/bin/python3 tools/recording_control.py stop --run-id trial_20261110_001
```

监听需显式开启。`auto` 和客户端默认使用 `$XDG_RUNTIME_DIR/dvp-monitor.sock`；变量不存在时使用 `/tmp/dvp-monitor-<uid>/control.sock`。可用监控端 `--control-socket /private/path/control.sock` 与客户端 `--socket` 指定相同绝对路径。直接父目录必须归当前用户所有且组/其他用户无访问权限；父目录不存在时按 0700 创建（上层目录需存在）。Socket 权限 0600，并检查对端 UID 相同。这是 Linux Unix-domain `SOCK_SEQPACKET`，不是 HTTP/TCP。远程推理主机需要通过另行安排的传输调用工作站侧客户端；本实现不开放网络服务。

在原构建依赖之外需要已安装的 `json-c` 开发文件。Python 客户端仅使用标准库，也可导入：`request(command, run_id=None, model=None, socket_path=None, timeout=3.0)` 返回响应字典。CLI 退出码：0 表示 `ok:true`，2 表示服务端拒绝，1 表示客户端/传输错误。**退出码 0 不等于录像正在进行，必须检查 `state`。**

## 请求与响应

每个连接发送一个 JSON 对象报文（最多 4096 字节），接收一个 JSON 响应。`version` 必须是整数 1。`command` 为 `start`、`stop` 或 `status`。start/stop 必须提供 `run_id`，由 1–100 个 ASCII 字母、数字、下划线或短横线组成；每次推理尝试使用新的唯一 ID。可选 `model` 为最多 1024 字节的字符串，start 重试必须保持一致。不接受调用者指定输出路径或 shell 命令。

```json
{"version":1,"command":"start","run_id":"trial_001","model":"checkpoint-A"}
```

成功响应包含 `ok`、`version`、`state`、`owner`（`manual`、`inference`、`none`）、`run_id`、`model`、绝对 `path`、`frames`、首末帧拷贝后的主机 Unix 时间、`reported_unix_ns`、`reason`、`synthetic` 和 AVI 标称 FPS。拒绝响应含 `ok:false`、`error`、`message`。不带运行 ID 的 status 返回当前归属；带 ID 时返回该运行的实时或持久化状态。status 依赖采集循环正常工作；相机故障可能关闭服务，不会必然返回健康结果。

- **开始确认：** 打开 AVI、向 OpenCV 提交首帧、刷新其 CSV 行并保存 `run.json` 后才返回。在推理执行点前检查 `ok:true`、`state:recording`、`frames>=1`、预期运行 ID 和 `synthetic:false`。OpenCV 写入错误报告有限：这是软件接收确认，不保证磁盘持久性或最终视频可解码。
- **停止确认：** 对活动且匹配的运行，释放视频写入器、关闭 CSV、原子替换清单后才返回。检查 `state:stopped`；`reason:requested` 区分显式停止和监控程序正常退出（`monitor_exit`）。预览保持打开。
- 对相同活动运行/模型重复 start 返回当前状态，不创建第二段。重复 stop 返回已保存结果。已完成、失败或中断的运行 ID 永不重新开始；start 可能返回非 recording 状态。新试验必须使用新 ID。
- 手动录像或其他运行占用时拒绝 start。其他 ID 的 stop 不会停止当前归属者。R/鼠标/stdin 不能停止推理拥有的片段；终端给出提示，预览显示 INFERENCE RECORD。Q/Esc/Ctrl-C 仍可退出监控程序，并以 `monitor_exit` 完成片段保存。
- 未知 ID、畸形/超大请求、错误版本以及 start 重试模型不匹配均拒绝。慢速/空闲客户端有限制（8 个客户端、1 秒报文等待），不会无限阻塞采集循环。

## 产物与故障处理

片段位于 `--recordings-dir/run-<run_id>/`，包含 `video.avi`、`frames.csv`、`run.json`。清单在开始确认和正常停止时写入，不逐帧刷新；活动运行的 `status` 是当前状态。客户端应将返回路径写进推理日志。帧拷贝后的 Unix 时间用于相同时钟下的近似对应，不代表硬件曝光或首条指令同步；CSV 也保留单调时钟时间。FPS 仅为标称回放时序。既有 SDK 自动曝光帧元数据限制仍适用。

客户端断连/丢失响应不撤销命令。超时后查询或重试**同一个运行 ID**，不要立即使用新 ID。客户端崩溃或 `SIGKILL` 不会自动停止录像：推理侧应使用 `try/finally` 及 SIGINT/SIGTERM 处理；孤立录像需要按其 ID 显式 stop 或退出监控程序。未实现超时租约或自动重新绑定。

监控程序正常退出会完成当前片段封装。崩溃可能留下不完整 AVI 和仍为 `recording` 的 `run.json`；重启后查询此非活动旧运行返回 `interrupted`，不会继续或冒充完成。已存在目录但没有可读清单时返回 `incomplete_run`，不复用。清单原子替换不等于 fsync/断电持久性保证。I/O/相机错误不代表录像成功。

第二个监听者永不删除已有 socket。异常崩溃后，先确认旧监控进程已退出，再手动删除残留 socket 并重启。不要删除活动监控程序的 socket。调用方应在自己的执行边界附近开始/停止，记录错误，并明确决定录像缺失是否中止推理启动。本接口不授予机器人运动权限。

## 验证

```bash
/usr/bin/python3 tests/control_test.py build/dvp_monitor
/usr/bin/python3 tests/recording_test.py build/dvp_monitor
ctest --test-dir build --output-on-failure
```

合成测试覆盖首帧确认、视频解码、重试、手动归属、运行冲突、畸形/超大/空闲客户端、监听冲突、正常退出、崩溃/重启及 start 响应丢失。真实推理到相机的联动仍由推理任务接入；测试未操作机器人。
