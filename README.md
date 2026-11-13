# DVP workstation monitor

[简体中文](README.zh-CN.md)

Standalone C++17 / OpenCV program for external live monitoring and optional recording.
No ROS, robot commands, or policy input. Original ROS source remains in Git history.

## Build

Requires CMake >= 3.18, a C++17 compiler, OpenCV development libraries, and the vendor's Linux x86-64 SDK. The supplied SDK stays local and is ignored by Git; keep its libraries and camera plugins together. Existing legacy SDK files under `include/` are retained but not used by this build. Do not mix SDK headers and binaries.

```bash
cmake -S . -B build -DDVP_SDK_ROOT="$PWD/DVP2-Linux64_231122/DVP2-Linux64"
cmake --build build -j4
```

Build RPATH points to the supplied SDK directory; CMake also symlinks its runtime libraries/plugins beside the executable for the vendor loader. No vendor installation script, kernel module, network configuration, ROS setup or root privilege is applied by this project. Camera-specific OS permissions and connectivity may still require separate setup.

## Use

Run from this repository. Start with device discovery, then select a printed friendly name if multiple cameras exist:

```bash
./build/dvp_monitor --list
./build/dvp_monitor --camera 'NAME'
./build/dvp_monitor --camera 'NAME' --record recordings-001 --fps 30
```

One camera is selected automatically if `--camera` is omitted. Q, Escape, window close or Ctrl-C stops capture and finalizes the video. `--headless` disables the window; `--duration 60` bounds the run. Recording starts immediately when `--record NEW_DIRECTORY` is supplied; its parent must exist and the directory must not already exist. Preview alone writes nothing.

New recordings default to `video.mp4` (H.264) and `frames.csv`. Use `--codec mjpeg` for the previous `video.avi` format. `--fps` sets the **output video FPS**, not the physical camera capture rate. Since 2026-11-12, host monotonic timestamps resample the video onto this fixed-rate timeline; playback duration follows elapsed recording time within one output frame. CSV records SDK frame ID/raw SDK timestamp and host monotonic/Unix timestamps after the image has been copied. Host timestamps are receive-side observations, not exposure timestamps; SDK timestamp units and clock relation are unverified. Synthetic records are explicitly marked.

Continuous capture requests BGR24 and disables external triggering for this open session; exposure and gain retain camera settings unless overridden; ROI retains camera settings. Unsupported formats/strides, SDK errors/timeouts, and resolution changes during recording fail explicitly. Stop/close is attempted on normal and exception exits. There is no automatic reconnection or frame replay.

This initial version performs acquisition, preview and encoding serially. Slow encoding/storage can delay acquisition; low latency and lossless capture are not guaranteed. SDK frame IDs help reveal gaps. OpenCV VideoWriter has limited write-error reporting: a finalized file is not a guarantee of complete footage; check disk space and decode recordings. No calibration or CameraInfo is fabricated.

## Offline checks

```bash
ctest --test-dir build --output-on-failure
/usr/bin/python3 tests/recording_test.py build/dvp_monitor
./build/dvp_monitor --synthetic --duration 10
```

The Python check requires system `cv2`. Synthetic checks exercise video decoding, metadata and overwrite protection without opening hardware. Actual camera discovery, GUI display, disconnect behavior and sustained capture/recording performance require workstation validation. The supplied SDK emits `LoadDbgConfig, LoadFile fail, err:-3` during startup in this environment; offline tests still pass, but this does not validate real camera operation.

## Workstation capture verification — 2026-11-10

The M3S130-H-O2C#U016500000370 was enumerated after the operator granted device-node access. A 10-second headless run produced 664 decodable 1280×1024 frames with matching CSV rows, 9.969 seconds between first/last host timestamps (66.506 FPS), and no SDK frame-ID gaps. AVI was encoded at nominal 60 FPS, so playback duration differs. Evidence is local under `recordings/headless-20261110-102529/verification.json`. The image is overexposed and sideways; exposure/orientation still need adjustment. This short run is not a sustained-performance or disconnect test.

SDK loading is now lazy: preload vendor camera plugins with Linux `RTLD_DEEPBIND`, then load `libdvp` and resolve its API. The vendor USB plugin embeds libusb; direct linking allowed OpenCV/gphoto's system libusb symbols to interpose and crashed in `libusb_get_next_timeout`. SDK and system libraries remain unchanged. Synthetic mode does not load SDK libraries. Preview treats unsupported visibility (`-1`) separately from an explicitly hidden window (`0`); use Q/Esc or Ctrl-C when a backend cannot report window closing.

After correcting the unsupported window-visibility result, a 10-second run with preview recorded and decoded all 666 frames (1280×1024, 66.570 observed FPS, zero SDK frame-ID gaps). Evidence: `recordings/preview-20261110-102645/verification.json`. The X11 window was observed. Q/Esc remains the portable exit method.

## Manual exposure — 2026-11-10

The current workstation scene was overexposed at SDK readback 15002.4 microseconds / gain 2 (auto exposure already off). Start with this tested setting:

```bash
./build/dvp_monitor --exposure-us 4000 --gain 1
```

Actual readback was 3997.68 microseconds / gain 1. A 3-second preview recording decoded all 211 frames. At the observed scene, 2 ms was dark and 6 ms clipped more highlights; 4 ms is a practical initial compromise, not automatic exposure or universal calibration. Camera placement changed during the user's setup, so the original sideways view is not a matched before/after comparison. No image rotation or brightness postprocessing is applied. Bright lamps may still clip.

Either manual flag disables automatic exposure; omitted flags leave the corresponding value unchanged. SDK rejects unsupported values. Settings were observed to survive close/reopen while connected; this program does not save nonvolatile device defaults, and persistence across power cycles is unverified. CSV now appends SDK-reported per-frame `exposure_us` and `analog_gain`. The selected camera settings affect other applications opening the same device later.

## Continuous automatic monitoring — 2026-11-10

```bash
./build/dvp_monitor --auto
# Optional recording:
./build/dvp_monitor --auto --record NEW_DIRECTORY --fps 60
```

`--auto` enables continuous automatic exposure AND analog gain (exposure priority) plus continuous automatic white balance. It neutralizes RGB processing gains to 1 and disables the color-temperature effect as required by the SDK white-balance documentation. Exposure ceiling is the smaller of the existing ceiling and 10000 microseconds; gain ceiling is the smaller of the existing ceiling and 4. The tested camera read back exposure 1–10000 microseconds and gain 1–3, AE mode 0, AE operation 2, AWB operation 2. Unsupported settings or readback mismatch fail explicitly. Automatic and manual flags cannot be combined. Metering region and brightness target remain camera settings; no ROI calibration is implied.

A 10-second recording decoded all 1018 frames. An 8-second dark-start recovery test decoded all 818 frames; grayscale mean rose from 18.70 to 95.68. This demonstrates brightness adaptation in this scene; continuous gain was enabled but stayed at 1, and white-balance mode was verified without controlled color-temperature testing. Bright lamps can still clip. No rotation or postprocessing was applied.

**SDK metadata limitation:** `frames.csv` preserves raw SDK `exposure_us`/`analog_gain`; exposure stayed constant in frame metadata during visible automatic recovery, so these fields are not authoritative per-frame applied settings in auto mode. Startup/final live API queries provide separate snapshots. Automatic settings may persist across reopening; this program does not call nonvolatile-save APIs. Use `--auto` explicitly when starting to reapply and verify the monitoring profile.

## USB permission recovery

USB bus/device numbers change after reconnecting; a one-off `setfacl` does not survive replacement of the device node. The supplied rule grants the active local desktop user access only to USB VID:PID `04b4:d003`. Install once, then physically unplug/replug the camera:

```bash
sudo install -m 0644 config/70-dvp-monitor.rules /etc/udev/rules.d/70-dvp-monitor.rules
sudo udevadm control --reload-rules
# Unplug/replug, then:
./build/dvp_monitor --list
./build/dvp_monitor --auto
```

These administrator commands are operator-run, not executed by this project. `uaccess` targets the active local desktop session, not headless/remote service accounts. For a temporary fix, use `lsusb -d 04b4:d003` to find the CURRENT bus/device and apply `sudo setfacl -m u:alienware:rw /dev/bus/usb/BUS/DEVICE`. Do not reuse an old USB node number.

The SDK may return successful enumeration but put `err:13,Permission denied` in `FriendlyName`. The monitor now rejects such entries and exits with a clear error without calling SDK open; explicitly selected names must also match an accessible enumeration entry. This avoids the observed permission-path crash; disconnect races inside the SDK are not thereby eliminated.

## Start/stop recording without restarting

```bash
./build/dvp_monitor --auto --fps 60
```

Click **START RECORD / STOP RECORD** at the top left, or focus the preview window and press **R**. The preview continues after stopping; each next start creates a new `recordings/YYYY-MM-DD_HH-MM-SS_mmm_manual/` folder with MP4 and CSV. A red REC status shows elapsed time and frame count; PREVIEW means nothing is being recorded. Overlays are display-only. Q/Esc/Ctrl-C finalizes any active clip and exits. `--recordings-dir PATH` selects the parent folder for interactive recordings; no folder is created until the first start.

`--record NEW_DIRECTORY` remains an optional immediate-start mode; stopping and restarting then creates subsequent clips under `--recordings-dir`. `--stdin-control` additionally accepts `r`/`q` followed by Enter, useful for terminal control and offline tests. Keyboard and mouse actions take effect at the next acquired frame. `--fps` controls real-time CFR output; capture can run faster or slower. A recording I/O error fails the program; a short synthetic test does not certify disk-full behavior. Existing processes need one restart to load this version; ordinary recording toggles do not restart the camera.

## Inference-triggered recording

Use `--control-socket auto` to enable the optional same-user local control service. [Protocol, client commands and failure handling](CONTROL_PROTOCOL.md). The build additionally requires `json-c` development files. Manual preview/recording remains available.

## H.264 recording default — 2026-11-10

New recordings use H.264 in MP4 (`video.mp4`) via the installed OpenCV FFmpeg backend; no silent codec fallback. Existing AVI clips are not converted or deleted. `--codec h264` explicitly selects the new default; `--codec mjpeg` selects the old format. Manual R/button controls and inference start/stop stay the same. Responses and `run.json` now add `codec` and absolute `video_file`; consumers should use that path rather than hard-code `video.avi`. Old manifests can lack these added fields.

This uses the local software encoder defaults (no explicit bitrate/CRF or GPU selection). H.264 is lossy; fine texture can lose detail. On the same 120 decoded low-motion real-camera frames, MJPEG was 18,281,348 bytes and H.264 358,392 bytes (98.04% smaller), with encode/finalize times 1.389 s and 1.218 s. Mean PSNR against the already-MJPEG-decoded source was 47.59 dB and 35.64 dB respectively; this is a comparative re-encode check, not raw-sensor fidelity or a universal size/speed guarantee. Evidence: `recordings/h264-check/comparison.json`.

Encoder buffering means first-frame submission acknowledgement is not proof of an immediately playable MP4; stop waits for encoder flush and MP4 finalization. SIGKILL/power loss can leave an incomplete MP4. The 2026-11-12 timeline correction below supersedes the old frame-per-capture timing. Codec verification (requires installed `ffprobe`):

```bash
/usr/bin/python3 tests/codec_test.py build/dvp_monitor
```

## Real-time playback correction — 2026-11-12

Previously every capture was written once at the chosen playback FPS: approximately 100 captures/s written at 60 FPS played at 0.6× speed. New recordings use `cfr_host_monotonic_v1`. First capture anchors time zero; one output slot per `1/--fps` second selects the first capture reaching that slot. Faster inputs are skipped; missing slots hold the most recently received image. Normal stop fills the final fractional interval up to the stop request time, before encoder finalization. Playback duration differs from first-capture-to-stop duration by at most one output frame. This uses host image-copy completion time, not hardware exposure time.

- `capture.csv`: every input capture during the clip, with original timestamps and SDK metadata.
- `frames.csv`: one row per encoded video frame, preserving its source timestamps plus `video_pts_ns`, `source_index` (row in capture.csv), and `held_frame` (1 for held imagery). Held images never receive a fabricated fresh source timestamp.
- Status/manifests retain `frames` as encoded video count and add `capture_frames`, `held_frames`, `timeline`, `recording_duration_s`. These counts can differ. Manual clips now also save `recording.json`; inference clips retain `run.json`.

Capture/encoding remain serial. A missing interval requiring more than 5 seconds of repeated output fails instead of creating an unbounded backlog or silently compressing time. Camera disconnect errors still stop the process; no fake continued camera health is implied. Existing files are not rewritten. Older manifests without `timeline` retain the old timing behavior.

`--synthetic-fps RATE` (synthetic mode only) independently sets test input rate. Deterministic tests cover 20/60/100/150 Hz input, irregular gaps and rejection of regressed/long gaps. End-to-end test:

```bash
/usr/bin/python3 tests/realtime_recording_test.py build/dvp_monitor
```

## Beijing-time directories — 2026-11-13

New directories use fixed Beijing time (UTC+08:00), independent of the system timezone, and sort by start time by name. Manual clips use `YYYY-MM-DD_HH-MM-SS_mmm_manual/`; inference clips use `YYYY-MM-DD_HH-MM-SS_mmm_inference-<run_id>/`. Millisecond collisions add a counter before the type to prevent overwriting. Explicit `--record PATH` keeps the supplied path. Old directories are not renamed; legacy `run-<run_id>/` supports queries, stop and idempotent retries. New directories remain discoverable by run ID after process restart. Clients must use returned `path` and `video_file`, not construct directory names.
