# DVP 工作站监控

[English](README.md)

独立 C++17 / OpenCV 程序，用于外部实时监控与可选录像。不依赖 ROS，不发送机器人指令，不接入策略输入。原 ROS 源码保留在 Git 历史中。

## 编译

需要 CMake >= 3.18、C++17 编译器、OpenCV 开发库和厂家 Linux x86-64 SDK。用户提供的 SDK 保留在本地并由 Git 忽略；保持其动态库和相机插件在一起。`include/` 中已有的旧 SDK 文件保留，但本次构建不使用。不要混用不同 SDK 的头文件和二进制库。

```bash
cmake -S . -B build -DDVP_SDK_ROOT="$PWD/DVP2-Linux64_231122/DVP2-Linux64"
cmake --build build -j4
```

构建 RPATH 指向所提供的 SDK 目录；CMake 还会将运行库/插件软链接到可执行文件旁，供厂家加载器查找。本项目不运行厂家安装脚本，不安装内核模块，不配置网络，不加载 ROS，不使用 root 权限。具体相机的系统权限和连接可能仍需单独配置。

## 使用

在本仓库目录执行。先枚举设备，多相机时指定输出的友好名称：

```bash
./build/dvp_monitor --list
./build/dvp_monitor --camera 'NAME'
./build/dvp_monitor --camera 'NAME' --record recordings-001 --fps 30
```

仅有一台相机时可省略 `--camera`。Q、Escape、关闭窗口或 Ctrl-C 停止采集并完成视频保存。`--headless` 关闭窗口；`--duration 60` 限定运行时间。提供 `--record NEW_DIRECTORY` 就立即开始录像；父目录必须存在，目标目录必须尚不存在。仅预览不会写文件。

每次录像包含 `video.avi`（MJPEG）和 `frames.csv`。MJPEG 文件可能较大。`--fps` 设置的是**标称回放帧率**，不是实际相机采集帧率。视频回放时长可能与实际采集时间不同。CSV 保存 SDK 帧号、原始 SDK 时间戳，以及图像拷贝后的主机单调时钟/Unix 时间戳。主机时间属于接收侧观测，不是曝光时刻；SDK 时间戳单位及其时钟关系尚未验证。模拟录像有明确标记。

连续采集请求 BGR24，并在本次打开期间关闭外部触发；曝光和增益在未显式指定参数时沿用相机设置；ROI 沿用相机设置。不支持的格式/行跨度、SDK 错误或超时、录像期间分辨率变化均明确报错。正常退出及异常退出均尝试停止和关闭相机。不自动重连，不回放旧帧。

初版串行执行采集、预览和编码。慢速编码/存储可能拖慢取帧，不保证低延迟或无丢帧；SDK 帧号可帮助发现缺帧。OpenCV VideoWriter 的写入错误报告有限：完成封装并不保证录像完整，需要检查磁盘空间和解码结果。不伪造标定或 CameraInfo。

## 离线检查

```bash
ctest --test-dir build --output-on-failure
/usr/bin/python3 tests/recording_test.py build/dvp_monitor
./build/dvp_monitor --synthetic --duration 10
```

Python 检查需要系统 `cv2`。模拟检查不打开硬件，覆盖视频解码、元数据和防覆盖。实际相机枚举、GUI 显示、断连行为及持续采集/录像性能仍需在工作站验证。此环境中所提供的 SDK 启动时输出 `LoadDbgConfig, LoadFile fail, err:-3`；离线测试仍通过，但这不代表真实相机运行已验证。

## 工作站采集验证 — 2026-11-10

操作员授予设备节点访问权限后，成功枚举 M3S130-H-O2C#U016500000370。10 秒无窗口采集得到 664 帧可解码的 1280×1024 图像，CSV 行数一致，首末主机时间戳间隔 9.969 秒（66.506 FPS），SDK 帧号未见跳帧。AVI 标称编码帧率为 60 FPS，因此回放时长不同。证据保存在本地 `recordings/headless-20261110-102529/verification.json`。画面过曝且方向侧转，曝光/方向仍需调整。本次短测不代表持续性能或断连验证。

SDK 改为延迟加载：先用 Linux `RTLD_DEEPBIND` 加载厂家相机插件，再加载 `libdvp` 并解析 API。厂家 USB 插件内嵌 libusb；直接链接时，OpenCV/gphoto 的系统 libusb 符号介入，导致在 `libusb_get_next_timeout` 崩溃。厂家 SDK 和系统库均未修改。模拟模式不加载 SDK 库。预览区分不支持可见性查询（`-1`）与明确隐藏的窗口（`0`）；后端无法报告窗口关闭时，使用 Q/Esc 或 Ctrl-C 退出。

修正不支持窗口可见性查询的返回值处理后，带预览的 10 秒运行录制并成功解码全部 666 帧（1280×1024，实测 66.570 FPS，SDK 帧号未见跳帧）。证据：`recordings/preview-20261110-102645/verification.json`。已观察到 X11 窗口。Q/Esc 是跨后端可靠的退出方式。

## 手动曝光 — 2026-11-10

当前工作站场景在 SDK 回读 15002.4 微秒 / 增益 2 时过曝（自动曝光原已关闭）。可使用已验证的起始设置：

```bash
./build/dvp_monitor --exposure-us 4000 --gain 1
```

实际回读为 3997.68 微秒 / 增益 1。3 秒带预览录像全部 211 帧解码通过。在观察的场景中，2 毫秒偏暗，6 毫秒高光溢出更多；4 毫秒是实用的初始折中，并非自动曝光或通用标定。用户布置过程中移动了相机，因此最初侧转画面不能作为严格配对的前后对比。没有旋转图像或做亮度后处理。明亮灯条仍可能饱和。

指定任一手动参数都会关闭自动曝光；未指定的参数保持原值。不支持的值由 SDK 拒绝。已观察到设置在保持连接、关闭再打开后仍保留；程序不保存非易失设备默认值，断电后的持久性未验证。CSV 新增逐帧 SDK 报告的 `exposure_us` 和 `analog_gain` 列。这些相机设置也会影响随后打开同一设备的其他应用。

## 连续全自动监控 — 2026-11-10

```bash
./build/dvp_monitor --auto
# 可选录像：
./build/dvp_monitor --auto --record NEW_DIRECTORY --fps 60
```

`--auto` 开启连续自动曝光和模拟增益（曝光优先），以及连续自动白平衡。按 SDK 白平衡说明，将 RGB 处理增益设为 1，并关闭色温效果。曝光上限取现有上限与 10000 微秒的较小值；增益上限取现有上限与 4 的较小值。实测相机回读为曝光 1–10000 微秒、增益 1–3、AE 模式 0、AE 操作 2、AWB 操作 2。不支持的设置或回读不一致会明确报错。自动与手动参数不能混用。测光区域和目标亮度沿用相机设置，不代表已做 ROI 标定。

10 秒录像全部 1018 帧解码通过。8 秒暗起点恢复测试全部 818 帧解码通过，灰度均值从 18.70 升至 95.68，证明此场景下亮度自动适应有效；自动增益已开启但保持 1，白平衡模式已回读确认，未做受控色温变化测试。明亮灯条仍可能饱和。未旋转或后处理画面。

**SDK 元数据限制：** `frames.csv` 保留 SDK 原始 `exposure_us`/`analog_gain`；可见自动恢复期间，帧元数据中的曝光却保持不变，因此自动模式下不能把这些列作为逐帧实际设置的权威值。启动/退出时直接查询 API 的结果属于独立状态快照。自动设置可能在重新打开后保留；本程序不调用非易失保存 API。每次启动显式使用 `--auto` 可重新应用并验证监控配置。

## USB 权限恢复

重新连接后 USB 总线/设备编号可能变化，单次 `setfacl` 不会在设备节点重建后保留。提供的规则仅为 USB VID:PID `04b4:d003` 授予当前本地图形桌面用户访问权。安装一次后，实际拔插相机：

```bash
sudo install -m 0644 config/70-dvp-monitor.rules /etc/udev/rules.d/70-dvp-monitor.rules
sudo udevadm control --reload-rules
# 拔插相机后：
./build/dvp_monitor --list
./build/dvp_monitor --auto
```

以上管理员命令由操作员执行，本项目未执行。`uaccess` 面向当前本地图形桌面会话，不面向无桌面/远程服务账户。临时恢复可先用 `lsusb -d 04b4:d003` 查当前总线/设备号，再执行 `sudo setfacl -m u:alienware:rw /dev/bus/usb/BUS/DEVICE`。不要复用旧设备编号。

SDK 可能返回枚举成功，却把 `err:13,Permission denied` 填入 `FriendlyName`。程序现在拒绝此类条目，明确报错退出，不调用 SDK 打开操作；显式指定的名称也必须匹配可访问的枚举条目。这避免了已观察到的权限分支崩溃，但不意味着消除了 SDK 内部的拔插竞态。

## 不重启程序即可开始/停止录像

```bash
./build/dvp_monitor --auto --fps 60
```

点击左上角 **START RECORD / STOP RECORD** 按钮，或先选中预览窗口再按 **R**。停止录像后预览继续；每次重新开始都会创建独立的 `recordings/capture-<Unix纳秒>/` 目录，包含 AVI 和 CSV。红色 REC 状态显示时长和帧数；PREVIEW 表示未录像。提示只显示在窗口，不写入视频。Q/Esc/Ctrl-C 保存当前录像并退出。`--recordings-dir PATH` 指定交互录像的父目录；第一次开始录像之前不创建目录。

`--record NEW_DIRECTORY` 仍可用于启动后立即录像；停止再开始的后续片段写入 `--recordings-dir`。额外加 `--stdin-control` 可在终端输入 `r`/`q` 后回车，便于终端控制和离线测试。按键/鼠标操作从下一张采集帧生效。`--fps` 仍为 AVI 标称回放帧率，不是采集时序。录像 I/O 错误会使程序失败退出；短时合成测试不能证明磁盘写满行为。旧进程只需重启一次加载新版，日常切换录像无需重启相机。

## 推理触发录像

使用 `--control-socket auto` 开启可选的同用户本地控制服务。[协议、客户端命令与故障处理](CONTROL_PROTOCOL.zh-CN.md)。构建额外需要 `json-c` 开发文件。仍保留手动预览和录像。
