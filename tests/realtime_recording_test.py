"""Real wall-clock synthetic recording, variable capture/output rates and stop flush."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import cv2

binary = str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory() as temp:
    for capture_fps, output_fps in [(100, 60), (60, 30), (20, 60)]:
        path = Path(temp) / f"{capture_fps}-{output_fps}"
        subprocess.run([binary, "--synthetic", "--headless", "--synthetic-fps", str(capture_fps),
                        "--fps", str(output_fps), "--duration", "1.5", "--record", str(path)], check=True, timeout=15)
        stats = json.loads((path / "recording.json").read_text())
        frames = list(csv.DictReader((path / "frames.csv").open()))
        captures = list(csv.DictReader((path / "capture.csv").open()))
        probe = json.loads(subprocess.check_output(["ffprobe", "-v", "error", "-select_streams", "v:0",
                           "-show_entries", "stream=duration,nb_frames", "-of", "json", str(path / "video.mp4")]))["streams"][0]
        duration = float(probe["duration"])
        assert abs(duration - stats["recording_duration_s"]) <= 1/output_fps + .00001
        assert stats["frames"] == len(frames) == int(probe["nb_frames"])
        assert stats["capture_frames"] == len(captures)
        assert stats["timeline"] == "cfr_host_monotonic_v1"
        if capture_fps > output_fps: assert len(captures) > len(frames)
        if capture_fps < output_fps: assert stats["held_frames"] > 0 and len(frames) > len(captures)
        for i, row in enumerate(frames):
            original = captures[int(row["source_index"])]
            assert row["host_receive_steady_ns"] == original["host_receive_steady_ns"]
            assert abs(int(row["video_pts_ns"]) - i*1e9/output_fps) <= 1
        cap = cv2.VideoCapture(str(path / "video.mp4")); decoded = 0
        while cap.read()[0]: decoded += 1
        cap.release(); assert decoded == len(frames)
        print(f"PASS: capture {capture_fps} -> output {output_fps}, {len(captures)} inputs/{len(frames)} encoded, real={stats['recording_duration_s']:.3f}s video={duration:.3f}s")
