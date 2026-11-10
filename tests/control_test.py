"""Black-box IPC and recording lifecycle regressions, synthetic sources only."""
import csv
import importlib.util
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import cv2

binary = str(Path(sys.argv[1]).resolve())
spec = importlib.util.spec_from_file_location("recording_control", Path(__file__).resolve().parents[1] / "tools/recording_control.py")
client = importlib.util.module_from_spec(spec)
spec.loader.exec_module(client)


def wait_for(predicate):
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.03)
    raise AssertionError("Timed out")


def decode(path):
    rows = list(csv.DictReader((Path(path) / "frames.csv").open()))
    cap = cv2.VideoCapture(str(Path(path) / "video.avi"))
    frames = 0
    while cap.read()[0]:
        frames += 1
    cap.release()
    assert frames == len(rows) and frames > 0, (frames, len(rows))
    return frames


with tempfile.TemporaryDirectory() as temporary:
    root = Path(temporary)
    endpoint = str(root / "control.sock")
    recordings = root / "recordings"
    def launch(*extra):
        p = subprocess.Popen([binary, "--synthetic", "--headless", "--stdin-control", "--fps", "20",
                              "--control-socket", endpoint, "--recordings-dir", str(recordings), *extra],
                             stdin=subprocess.PIPE, text=True)
        wait_for(lambda: Path(endpoint).exists() or p.poll() is not None)
        assert p.poll() is None
        return p

    def call(command, run=None, model=None):
        return client.request(command, run, model, endpoint)

    p = launch()
    try:
        assert call("status")["state"] == "idle"
        # Binding a second monitor must not steal/delete the live endpoint.
        other = subprocess.run([binary, "--synthetic", "--headless", "--control-socket", endpoint], timeout=5)
        assert other.returncode == 1 and call("status")["state"] == "idle"
        for raw in (b"not json", b'{}', b'{"version":99,"command":"status"}', b"x" * 5000):
            with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as channel:
                channel.settimeout(3)
                channel.connect(endpoint); channel.sendall(raw)
                assert not json.loads(channel.recv(16384))["ok"]
        assert not call("start", "../escape")["ok"]
        # A silent connection cannot block capture or another client.
        silent = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET); silent.connect(endpoint)
        start = call("start", "trial_1", "checkpoint-A")
        assert start["ok"] and start["state"] == "recording" and start["frames"] >= 1
        assert start["first_frame_unix_ns"] > 0
        again = call("start", "trial_1", "checkpoint-A")
        assert again["path"] == start["path"]
        assert not call("start", "trial_1", "checkpoint-B")["ok"]
        assert not call("start", "trial_2", "checkpoint-B")["ok"]
        assert not call("stop", "wrong_run")["ok"]
        p.stdin.write("r\n"); p.stdin.flush(); time.sleep(0.15)
        assert call("status")["run_id"] == "trial_1", "R must not stop external recording"
        stop = call("stop", "trial_1")
        assert stop["ok"] and stop["state"] == "stopped"
        count = decode(stop["path"])
        assert count == stop["frames"]
        assert call("stop", "trial_1")["frames"] == count
        assert call("start", "trial_1", "checkpoint-A")["state"] == "stopped"
        assert call("status")["state"] == "idle"
        silent.close()
        # Manual recording is owned independently.
        p.stdin.write("r\n"); p.stdin.flush()
        wait_for(lambda: call("status")["owner"] == "manual")
        assert not call("start", "trial_manual_conflict")["ok"]
        assert not call("stop", "trial_manual_conflict")["ok"]
        assert call("stop", "trial_1")["state"] == "stopped"
        assert call("status")["owner"] == "manual"
        p.stdin.write("r\n"); p.stdin.flush()
        wait_for(lambda: call("status")["state"] == "idle")
        # Clean process exit finalizes run and records a distinct reason.
        active = call("start", "shutdown", "checkpoint-A")
        p.send_signal(signal.SIGINT); assert p.wait(timeout=5) == 0
        state = json.loads((Path(active["path"]) / "run.json").read_text())
        assert state["state"] == "stopped" and state["reason"] == "monitor_exit"
        decode(active["path"])
        p = launch()
        assert call("status", "trial_1")["state"] == "stopped"
        # Unexpected exit leaves run explicitly interrupted, never replayed.
        interrupted = call("start", "crashed", "checkpoint-A")
        p.kill(); p.wait(timeout=5)
        assert Path(endpoint).exists(), "Crash must not silently free another owner's socket"
        Path(endpoint).unlink()  # Test operator cleanup after proven dead process only.
        p = launch()
        assert call("status", "crashed")["state"] == "interrupted"
        assert call("start", "crashed", "checkpoint-A")["state"] == "interrupted"
        assert call("status")["state"] == "idle"
        # Lost response: disconnect after send; same run query/retry resolves outcome.
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as channel:
            channel.connect(endpoint)
            channel.sendall(b'{"version":1,"command":"start","run_id":"lost_reply","model":"A"}')
        wait_for(lambda: call("status", "lost_reply").get("state") == "recording")
        assert call("start", "lost_reply", "A")["state"] == "recording"
        assert call("stop", "lost_reply")["state"] == "stopped"
        print("PASS: control lifecycle, first-frame ack, idempotence, ownership, malformed/idle clients, restart, crash and lost reply")
    finally:
        if p.poll() is None:
            p.send_signal(signal.SIGINT); p.wait(timeout=5)
        p.stdin.close()
