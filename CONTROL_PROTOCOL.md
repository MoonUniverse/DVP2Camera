# Local inference recording control — v1

[简体中文](CONTROL_PROTOCOL.zh-CN.md)

Camera-only interface. No ROS, robot commands, inference inputs or physical synchronization guarantees. The inference owner integrates its own startup/finally paths; this package never launches inference.

## Launch and client

```bash
./build/dvp_monitor --auto --fps 60 --control-socket auto
/usr/bin/python3 tools/recording_control.py status
/usr/bin/python3 tools/recording_control.py start --run-id trial_20261110_001 --model checkpoint-A
/usr/bin/python3 tools/recording_control.py status --run-id trial_20261110_001
/usr/bin/python3 tools/recording_control.py stop --run-id trial_20261110_001
```

The listener is opt-in. `auto` and the client resolve `$XDG_RUNTIME_DIR/dvp-monitor.sock`, or `/tmp/dvp-monitor-<uid>/control.sock` when that variable is absent. Override with monitor `--control-socket /private/path/control.sock` and client `--socket` using the same absolute path. The immediate parent must be owned by the current user with no group/other access; a missing parent is created mode 0700 (its ancestors must exist). Socket mode is 0600 and peer UID must match. This is a Linux Unix-domain `SOCK_SEQPACKET` endpoint, not HTTP/TCP. A remote inference host must call a workstation-side client via an independently arranged transport; this implementation does not expose a network service.

Requires installed `json-c` development files in addition to the existing build dependencies. The Python client uses only the standard library and can also be imported: `request(command, run_id=None, model=None, socket_path=None, timeout=3.0)` returns a response dictionary. CLI exit codes: 0 for `ok:true`, 2 for a server rejection, 1 for client/transport error. **Exit code 0 alone does not mean a recording is active: inspect `state`.**

## Requests and replies

One JSON object packet (maximum 4096 bytes) and one JSON reply per connection. Required `version` is integer 1. `command` is `start`, `stop` or `status`. `run_id` is required for start/stop, 1–100 ASCII letters/digits/underscore/dash; choose a new unique ID for every inference attempt. Optional `model` is a string of at most 1024 bytes, provided on start and identically on start retries. No caller-supplied output paths or shell commands are accepted.

```json
{"version":1,"command":"start","run_id":"trial_001","model":"checkpoint-A"}
```

Success replies contain `ok`, `version`, `state`, `owner` (`manual`, `inference`, `none`), `run_id`, `model`, absolute `path`, `frames`, first/last host frame-copy Unix timestamps, `reported_unix_ns`, `reason`, `synthetic`, and nominal video FPS. Rejections contain `ok:false`, `error`, `message`. Status without a run ID reports current ownership; with a run ID it returns that run's active or persisted state. Status requires a working capture loop; camera failure can close the service rather than return a healthy result.

- **Start acknowledgement:** returned only after opening the video, submitting the first image to OpenCV, flushing its CSV row and saving `run.json`. Require `ok:true`, `state:recording`, `frames>=1`, expected run ID and `synthetic:false` before the inference execution point. OpenCV has limited write-error reporting: this is software acceptance, not a guarantee of disk durability or eventual video decode.
- **Stop acknowledgement:** for the active matching run, returned after writer release, CSV close and atomic manifest replacement. Require `state:stopped`; `reason:requested` distinguishes explicit stop from normal monitor shutdown (`monitor_exit`). Preview remains open.
- Repeated start for the same active run/model returns its current state without opening a second clip. Repeated stop returns the stored result. Reusing a completed, failed or interrupted run ID never starts it again; start can return a non-recording state. Use a fresh ID for a new trial.
- Busy manual recording or another active run rejects start. Stop for another ID cannot stop the current owner. R/mouse/stdin toggles cannot stop an inference-owned clip; terminal prints a message and preview says INFERENCE RECORD. Q/Esc/Ctrl-C may still close the monitor and finalize the clip with `monitor_exit`.
- Unknown IDs, malformed/oversized requests, wrong versions and mismatched model on start retries are rejected. Slow/idle clients are bounded (8 clients, 1-second packet wait) and do not wait on the capture loop indefinitely.

## Artifacts and failure handling

Clips are under `--recordings-dir/YYYY-MM-DD_HH-MM-SS_mmm_inference-<run_id>/`, containing `video.mp4`, `frames.csv` and `run.json`. The manifest is written at acknowledged start and clean stop, not on every frame; active `status` is current. The client must save the returned path with the inference log. Frame-copy Unix times support approximate same-clock correlation, not hardware exposure or first-command synchronization; CSV also retains monotonic time. FPS defines the real-time CFR output timeline (see dated correction below). Existing SDK auto-exposure frame-metadata limitations remain.

A client disconnect/lost reply does not roll back a command. On timeout, query/retry **the same run ID**; never immediately issue a fresh ID. A client crash or `SIGKILL` does not auto-stop recording: the inference side should use `try/finally` plus SIGINT/SIGTERM handling; an orphan recording requires explicit stop with its run ID or monitor shutdown. No timeout lease or automatic reattachment is implemented.

Normal monitor exit finalizes its active clip. A monitor crash can leave video incomplete and `run.json` at `recording`; after restart, status reports that inactive old run as `interrupted`, never resumed or falsely completed. An existing directory without a readable manifest is `incomplete_run` and not reused. Atomic manifest replacement is not an fsync/power-loss durability guarantee. I/O/camera errors are not recording success.

A second listener never removes an existing socket. After an unexpected crash, first establish that the old monitor process is dead, then manually remove the stale socket and restart. Do not remove a live monitor's socket. The caller must start/stop near its own execution boundaries, record errors, and decide explicitly whether missing recording should abort an inference launch. This interface grants no robot motion permission.

## Verification

```bash
/usr/bin/python3 tests/control_test.py build/dvp_monitor
/usr/bin/python3 tests/recording_test.py build/dvp_monitor
ctest --test-dir build --output-on-failure
```

Synthetic tests cover first-frame acknowledgement, video decode, retries, manual ownership, conflicting runs, malformed/oversized/idle clients, listener conflict, normal exit, crash/restart, and a lost start reply. Live inference-to-camera integration remains for the inference task; no robot was operated by these tests.

H.264 update: new clips default to H.264 MP4. Responses/manifests add `codec` (`h264` or `mjpeg`) and absolute `video_file`. Use the returned path; `--codec mjpeg` writes `video.avi`, and older manifests can omit these fields. Start confirms submission to the encoder, not a playable MP4; stop flushes the encoder and finalizes the container. Request protocol version remains 1.

## Timeline correction — 2026-11-12

New clips declare `timeline:cfr_host_monotonic_v1`. `nominal_fps` is now the real-time CFR output rate, independent of capture speed. `frames` remains the encoded video count; `capture_frames` counts all input samples, and `held_frames` counts held-image outputs. `recording_duration_s` measures first capture to stop (or latest capture while active), excluding encoder finalization. Video duration differs by at most one output period.

`capture.csv` retains all captures. `frames.csv` aligns with decoded output and adds `video_pts_ns`, `source_index`, `held_frame`; original source timestamps can repeat for held images. Do not interpret repeats as fresh captures. `first_frame_unix_ns` / `last_frame_unix_ns` describe source captures, not the padded end of the output. Stop replies include the final resampled frame count after tail padding and flush. Request version and ownership semantics remain unchanged. Old files without the timeline marker are not repaired automatically. A catch-up gap over 5 seconds errors instead of unbounded duplication.

## Beijing-time directories — 2026-11-13

New directories use fixed Beijing time (UTC+08:00), independent of the system timezone, and sort by start time by name. Manual clips use `YYYY-MM-DD_HH-MM-SS_mmm_manual/`; inference clips use `YYYY-MM-DD_HH-MM-SS_mmm_inference-<run_id>/`. Millisecond collisions add a counter before the type to prevent overwriting. Explicit `--record PATH` keeps the supplied path. Old directories are not renamed; legacy `run-<run_id>/` supports queries, stop and idempotent retries. New directories remain discoverable by run ID after process restart. Clients must use returned `path` and `video_file`, not construct directory names.
