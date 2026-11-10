#!/usr/bin/env python3
"""Local DVP recorder client; no ROS, SDK or robot-control dependency."""
import argparse
import json
import os
from pathlib import Path
import socket
import sys


def default_socket():
    runtime = os.environ.get("XDG_RUNTIME_DIR")
    return str(Path(runtime) / "dvp-monitor.sock") if runtime else f"/tmp/dvp-monitor-{os.getuid()}/control.sock"


def request(command, run_id=None, model=None, socket_path=None, timeout=3.0):
    """One request. Timeout has unknown outcome; query/retry the SAME run_id."""
    payload = {"version": 1, "command": command}
    if run_id is not None:
        payload["run_id"] = run_id
    if model is not None:
        payload["model"] = model
    raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode()
    if len(raw) > 4096:
        raise ValueError("Control request exceeds 4096 bytes")
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as channel:
        channel.settimeout(timeout)
        channel.connect(socket_path or default_socket())
        channel.sendall(raw)
        response, _, flags, _ = channel.recvmsg(16384)
        if flags & socket.MSG_TRUNC or not response:
            raise RuntimeError("Missing/truncated response; outcome unknown")
    result = json.loads(response)
    if not isinstance(result, dict) or not isinstance(result.get("ok"), bool):
        raise RuntimeError("Invalid recorder response")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("start", "stop", "status"))
    parser.add_argument("--run-id")
    parser.add_argument("--model")
    parser.add_argument("--socket", default=default_socket())
    parser.add_argument("--timeout", type=float, default=3.0)
    args = parser.parse_args()
    if args.command != "status" and not args.run_id:
        parser.error("start/stop require --run-id")
    try:
        result = request(args.command, args.run_id, args.model, args.socket, args.timeout)
    except (OSError, ValueError, RuntimeError) as error:
        print(json.dumps({"ok": False, "error": "transport_or_client_error", "message": str(error),
                          "outcome": "unknown; query same run_id before retry"}), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
