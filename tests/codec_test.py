"""Verify actual encoded format and final frame count, including MJPEG fallback."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import cv2

binary = str(Path(sys.argv[1]).resolve())
with tempfile.TemporaryDirectory() as temp:
    for requested, filename, expected in [(None, "video.mp4", "h264"), ("mjpeg", "video.avi", "mjpeg")]:
        path = Path(temp) / expected
        command = [binary, "--synthetic", "--headless", "--fps", "20", "--duration", "0.5", "--record", str(path)]
        if requested:
            command.extend(["--codec", requested])
        subprocess.run(command, check=True, timeout=15)
        probe = json.loads(subprocess.check_output([
            "ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
            "stream=codec_name,pix_fmt", "-of", "json", str(path / filename)], text=True))
        assert probe["streams"][0]["codec_name"] == expected
        if expected == "h264":
            assert probe["streams"][0]["pix_fmt"] == "yuv420p"
        capture = cv2.VideoCapture(str(path / filename))
        count = 0
        while capture.read()[0]:
            count += 1
        capture.release()
        rows = list(csv.DictReader((path / "frames.csv").open()))
        assert count == len(rows) and count >= 2
        print(f"PASS: {expected} actual codec, {count} matching decoded frames/CSV rows")
