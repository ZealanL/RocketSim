#!/usr/bin/env python3
"""vis-v2 isolated debug-render review harness (no renderer here)."""

import argparse
import os
import pathlib
import signal
import socket
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve()
ROOT = HERE.parents[1]
ARTIFACTS = (ROOT / "target/input-review").resolve()
OUT_DEFAULT = ARTIFACTS / "debug-render"
SCENARIOS = ("wire", "solid", "showcase", "stress")


def artifact_path(value):
    path = (ROOT / value).resolve()
    if path != ARTIFACTS and ARTIFACTS not in path.parents:
        raise ValueError("output must stay under target/input-review")
    return path


def log(msg):
    print(f"vis-v2: {msg}", flush=True)


def display_busy(display):
    probe = subprocess.run(
        ["xwininfo", "-display", display, "-root"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return probe.returncode == 0


def pick_display():
    for num in range(99, 150):
        name = f":{num}"
        if not display_busy(name):
            return name
    raise RuntimeError("no unused X display in :99-:149")


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def wait_listen(host, port, deadline_s):
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        try:
            with socket.create_connection((host, port), timeout=0.3):
                return True
        except OSError:
            time.sleep(0.1)
    return False


def stop_proc(proc, timeout=3):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=2)


def capture_png(display, out_png):
    out_png.parent.mkdir(parents=True, exist_ok=True)
    tmp = out_png.with_suffix(".xwd")
    subprocess.run(
        ["xwd", "-display", display, "-root", "-out", str(tmp)], check=True
    )
    try:
        subprocess.run(
            [sys.executable, str(ROOT / "tools/xwd2png.py"), str(tmp), str(out_png)],
            check=True,
        )
    finally:
        tmp.unlink(missing_ok=True)
    log(f"captured {out_png}")


def parse_geometry(text):
    try:
        wide, high = text.lower().split("x")
        wide, high = int(wide), int(high)
    except ValueError:
        raise argparse.ArgumentTypeError("geometry must be WxH, e.g. 1280x720")
    if not (320 <= wide <= 3840 and 200 <= high <= 2160):
        raise argparse.ArgumentTypeError("geometry out of range")
    return f"{wide}x{high}"


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--viewer", default="target/release/rocketsim-viewer")
    parser.add_argument("--demo", default="target/release/examples/debug_render")
    parser.add_argument("--obj", default=None, help="Zero Two .obj for wire/solid")
    parser.add_argument("--out-dir", default="target/input-review/debug-render")
    parser.add_argument("--display", default=None, help="unused Xephyr display (auto if omitted)")
    parser.add_argument("--geometry", type=parse_geometry, default="1280x720")
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--seconds", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--settings", default=None)
    parser.add_argument("--cam", default="birds")
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=0)
    parser.add_argument("--count", type=int, default=500)
    parser.add_argument("--stress-count", type=int, default=5000)
    parser.add_argument("--static", action="store_true")
    parser.add_argument("--drive", action="store_true", help="Use the drivable Soccar scene")
    parser.add_argument("--show-ball", action="store_true", help="Keep the simulation ball visible in the debug scene")
    parser.add_argument("--viewer-arg", action="append", default=[])
    parser.add_argument("--only", action="append", default=[], choices=list(SCENARIOS))
    return parser


def demo_args(args, scenario, port, obj_path):
    cmd = [
        args.demo,
        "--addr",
        f"127.0.0.1:{port}",
        "--seconds",
        str(args.seconds),
        "--mode",
        scenario,
        "--count",
        str(args.stress_count if scenario == "stress" else args.count),
    ]
    if scenario in ("wire", "solid"):
        cmd += ["--obj", str(obj_path)]
    if args.static:
        cmd += ["--static"]
    if args.drive:
        cmd += ["--drive"]
    return cmd


def start_viewer(args, port, metrics_path, env, viewer_log):
    extra = list(args.viewer_arg)
    if "--no-menu" not in extra:
        extra.append("--no-menu")
    if not args.show_ball and not args.drive and "--hide-ball" not in extra:
        extra.append("--hide-ball")
    base = [args.viewer, str(port), args.cam, "--bind", args.bind] + extra
    proc = subprocess.Popen(
        base + ["--debug-stats", str(metrics_path)],
        env=env,
        stdout=viewer_log,
        stderr=subprocess.STDOUT,
    )
    time.sleep(1.0)
    if proc.poll() is not None:
        raise RuntimeError(f"viewer exited ({proc.returncode}); inspect viewer.log")
    return proc


def run_one(args, scenario, display, host_env, out_dir, obj_path, settings_bytes):
    port = args.port if args.port else free_port()
    work = out_dir / scenario
    config = work / "config"
    (config / "rocketsim-vis").mkdir(parents=True, exist_ok=True)
    if settings_bytes is None:
        if scenario in ("wire", "solid"):
            position = "[3500.0, -450.0, 1800.0]" if args.drive else "[2800.0, -300.0, 1000.0]"
            fov = 90.0 if args.drive else 65.0
            camera = f"fov_degrees = {fov}\nbirds_eye_pos = {position}\n"
        else:
            camera = "fov_degrees = 70.0\nbirds_eye_pos = [-2800.0, -3300.0, 2300.0]\n"
        settings_bytes = (
            "fps_limit_hz = 0\nvsync = false\nmsaa = 4\n"
            "show_trail = false\nshow_boost = false\nshow_ground_line = false\nshow_pads = false\n[camera]\n"
            + camera
        ).encode()
    (config / "rocketsim-vis" / "settings.toml").write_bytes(settings_bytes)
    for name in ("cache", "data"):
        (out_dir / name).mkdir(parents=True, exist_ok=True)
    env = host_env.copy()
    env["DISPLAY"] = display
    env["XDG_CONFIG_HOME"] = str(config.resolve())
    env["XDG_CACHE_HOME"] = str((out_dir / "cache").resolve())
    env["XDG_DATA_HOME"] = str((out_dir / "data").resolve())

    shot = work / f"{scenario}.png"
    metrics = work / "metrics.json"
    if metrics.exists():
        metrics.unlink()
    viewer = demo = None
    try:
        with (work / "viewer.log").open("wb") as viewer_log, (work / "demo.log").open(
            "wb"
        ) as demo_log:
            viewer = start_viewer(args, port, metrics, env, viewer_log)
            if not wait_listen("127.0.0.1", port, 5.0):
                if viewer.poll() is not None:
                    raise RuntimeError(f"viewer exited ({viewer.returncode}); see {scenario}/viewer.log")
                raise RuntimeError("viewer did not listen in time")
            demo = subprocess.Popen(
                demo_args(args, scenario, port, obj_path),
                env=env,
                stdout=demo_log,
                stderr=subprocess.STDOUT,
            )
            ready_deadline = time.monotonic() + args.timeout
            while b"READY " not in (work / "demo.log").read_bytes():
                if viewer.poll() is not None:
                    raise RuntimeError(f"viewer exited ({viewer.returncode}) before scene setup")
                if demo.poll() is not None:
                    raise RuntimeError(f"demo exited ({demo.returncode}) before scene setup")
                if time.monotonic() >= ready_deadline:
                    raise RuntimeError("demo did not finish scene setup in time")
                time.sleep(0.1)
            end_warm = time.monotonic() + args.warmup
            while time.monotonic() < end_warm:
                if viewer.poll() is not None:
                    raise RuntimeError(f"viewer exited ({viewer.returncode}) before capture")
                if demo.poll() is not None and demo.returncode != 0:
                    raise RuntimeError(f"demo exited ({demo.returncode}) before capture")
                time.sleep(0.2)
            capture_png(display, shot)
            try:
                demo.wait(timeout=max(1.0, args.timeout - args.warmup))
            except subprocess.TimeoutExpired:
                raise RuntimeError("demo did not exit in time")
            if demo.returncode != 0:
                raise RuntimeError(f"demo exited ({demo.returncode})")
    finally:
        stop_proc(demo)
        stop_proc(viewer)
    if metrics.is_file():
        log(f"{scenario}: metrics {metrics.stat().st_size} bytes")
    else:
        log(f"{scenario}: no metrics.json (needs viewer --debug-stats)")
    return True


def main():
    parser = build_parser()
    args = parser.parse_args()
    if not 0 <= args.warmup <= 30:
        parser.error("--warmup must be 0..30")
    if not 1 <= args.seconds <= 60:
        parser.error("--seconds must be 1..60")
    if args.warmup >= args.seconds:
        parser.error("--warmup must be less than --seconds")
    if not 10 <= args.timeout <= 180:
        parser.error("--timeout must be 10..180")
    if not 1 <= args.count <= 200000:
        parser.error("--count out of range")
    if not 1 <= args.stress_count <= 200000:
        parser.error("--stress-count out of range")
    if args.port and not 1 <= args.port <= 65535:
        parser.error("--port must be 1..65535 or 0 for ephemeral")
    selected = args.only or list(SCENARIOS)

    try:
        out_dir = artifact_path(args.out_dir)
    except ValueError as error:
        parser.error(str(error))
    out_dir.mkdir(parents=True, exist_ok=True)

    viewer_bin = (ROOT / args.viewer).resolve()
    demo_bin = (ROOT / args.demo).resolve()
    missing = [str(p) for p in (viewer_bin, demo_bin) if not p.is_file()]
    if missing:
        log(f"missing binaries: {', '.join(missing)}")
        log("build viewer and demo first; skip GPU run")
        raise SystemExit(2)

    obj_path = None
    if any(s in selected for s in ("wire", "solid")):
        if not args.obj:
            log("wire/solid need --obj PATH to a local Zero Two .obj")
            raise SystemExit(2)
        obj_path = pathlib.Path(args.obj).resolve()
        if not obj_path.is_file():
            log(f"obj not found: {obj_path}")
            raise SystemExit(2)

    settings_bytes = None
    if args.settings:
        source = pathlib.Path(args.settings).resolve()
        if not source.is_file():
            parser.error(f"settings file does not exist: {source}")
        settings_bytes = source.read_bytes()

    display = args.display or pick_display()
    if not display.startswith(":"):
        parser.error("--display must look like :99")
    if display_busy(display):
        raise RuntimeError(f"refusing to reuse active display {display}")

    host_env = os.environ.copy()
    host_env.pop("WAYLAND_DISPLAY", None)
    server = None
    try:
        with (out_dir / "xephyr.log").open("ab") as xephyr_log:
            server = subprocess.Popen(
                ["Xephyr", display, "-screen", args.geometry, "-nolisten", "tcp"],
                stdout=subprocess.DEVNULL,
                stderr=xephyr_log,
                env=host_env,
            )
            end = time.monotonic() + 8
            while time.monotonic() < end:
                if server.poll() is not None:
                    raise RuntimeError(f"Xephyr exited ({server.returncode})")
                if not display_busy(display):
                    time.sleep(0.1)
                    continue
                break
            else:
                raise RuntimeError("Xephyr did not become ready in 8s")
        log(f"Xephyr on {display} {args.geometry}")
        failed = []
        for scenario in selected:
            log(f"start {scenario}")
            try:
                run_one(args, scenario, display, host_env, out_dir, obj_path, settings_bytes)
                log(f"done {scenario}")
            except Exception as error:
                log(f"fail {scenario}: {error}")
                failed.append(scenario)
        if failed:
            raise SystemExit(1)
        log(f"artifacts under {out_dir}")
    finally:
        stop_proc(server, timeout=5)


if __name__ == "__main__":
    main()
