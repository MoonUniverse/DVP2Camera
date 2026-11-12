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
    assert all(a <= b for a, b in zip(times, times[1:]))
    with (recording / "capture.csv").open() as stream:
        capture_times = [int(row["host_receive_steady_ns"]) for row in csv.DictReader(stream)]
    assert all(a < b for a, b in zip(capture_times, capture_times[1:]))
    capture = cv2.VideoCapture(str(recording / "video.mp4"))
    decoded = 0
    while True:
        success, frame = capture.read()
        if not success:
            break
        assert frame.shape == (480, 640, 3)
        decoded += 1
    capture.release()
    assert decoded == len(rows), (decoded, len(rows))
    original = (recording / "video.mp4").read_bytes()
    assert subprocess.run(command, timeout=20).returncode != 0
    assert (recording / "video.mp4").read_bytes() == original
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
        capture = cv2.VideoCapture(str(interrupted / "video.mp4"))
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

    # Same input toggle handler as the GUI: two recordings, one continuous process.
    clips = pathlib.Path(directory) / "clips"
    process = subprocess.Popen(
        [binary, "--synthetic", "--headless", "--stdin-control",
         "--recordings-dir", str(clips), "--fps", "20"],
        stdin=subprocess.PIPE, text=True,
    )
    def send(command):
        process.stdin.write(command + "\n")
        process.stdin.flush()

    def wait_frames(folder, minimum):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            matches = sorted(folder.glob("capture-*/frames.csv"))
            if matches:
                lines = matches[-1].read_text().splitlines()
                if len(lines) >= minimum + 1:
                    return matches[-1]
            assert process.poll() is None
            time.sleep(0.02)
        raise AssertionError("Recording did not produce frames")

    try:
        time.sleep(0.3)
        assert not clips.exists(), "Preview must not create recordings"
        send("r")
        first_csv = wait_frames(clips, 4)
        send("r")
        time.sleep(0.2)
        saved = first_csv.read_bytes()
        time.sleep(0.2)
        assert first_csv.read_bytes() == saved and process.poll() is None
        send("r")
        deadline = time.monotonic() + 5
        while len(list(clips.glob("capture-*"))) < 2:
            assert time.monotonic() < deadline
            time.sleep(0.02)
        second_csv = wait_frames(clips, 4)
        assert second_csv != first_csv
        send("q")  # Exiting during recording must finalize the second clip.
        assert process.wait(timeout=5) == 0
        for sidecar in (first_csv, second_csv):
            rows = list(csv.DictReader(sidecar.open()))
            assert [int(row["index"]) for row in rows] == list(range(len(rows)))
            capture = cv2.VideoCapture(str(sidecar.parent / "video.mp4"))
            decoded = 0
            while True:
                success, frame = capture.read()
                if not success:
                    break
                # GUI overlay location must retain the synthetic source background.
                assert max(abs(int(x) - y) for x, y in zip(frame[20, 20], (35, 40, 45))) < 10
                decoded += 1
            capture.release()
            assert decoded == len(rows) and decoded >= 4
        print("PASS: preview/start/stop/restart/exit, two independent decodable clips")
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        process.stdin.close()
