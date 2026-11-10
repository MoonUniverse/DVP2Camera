"""Offline black-box checks; never enumerate or open real cameras."""
import csv
import pathlib
import signal
import time
import subprocess
import sys
import tempfile

import cv2

binary = str(pathlib.Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory() as directory:
    recording = pathlib.Path(directory) / "capture"
    command = [binary, "--synthetic", "--headless", "--fps", "20",
               "--duration", "0.5", "--record", str(recording)]
    subprocess.run(command, check=True, timeout=20)
    with (recording / "frames.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) >= 2
    assert all(row["synthetic"] == "1" for row in rows)
    times = [int(row["host_receive_steady_ns"]) for row in rows]
    assert all(a < b for a, b in zip(times, times[1:]))
    capture = cv2.VideoCapture(str(recording / "video.avi"))
    decoded = 0
    while True:
        success, frame = capture.read()
        if not success:
            break
        assert frame.shape == (480, 640, 3)
        decoded += 1
    capture.release()
    assert decoded == len(rows), (decoded, len(rows))
    original = (recording / "video.avi").read_bytes()
    assert subprocess.run(command, timeout=20).returncode != 0
    assert (recording / "video.avi").read_bytes() == original
    print(f"PASS: {decoded} decoded frames, timestamps, overwrite protection")

    interrupted = pathlib.Path(directory) / "interrupted"
    process = subprocess.Popen([binary, "--synthetic", "--headless", "--record", str(interrupted)])
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            sidecar = interrupted / "frames.csv"
            if sidecar.exists() and len(sidecar.read_text().splitlines()) >= 4:
                break
            if process.poll() is not None:
                raise AssertionError("Capture exited before SIGINT")
            time.sleep(0.05)
        else:
            raise AssertionError("No frames before deadline")
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=5) == 0
        with sidecar.open() as stream:
            expected = len(list(csv.DictReader(stream)))
        capture = cv2.VideoCapture(str(interrupted / "video.avi"))
        decoded = 0
        while capture.read()[0]:
            decoded += 1
        capture.release()
        assert decoded == expected and decoded >= 3
        print(f"PASS: SIGINT finalized {decoded} decodable frames")
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
