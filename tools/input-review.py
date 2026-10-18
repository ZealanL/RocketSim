#!/usr/bin/env python3
"""Bounded X11/XTest review helper for the RocketSim viewer."""

import argparse
import ctypes
import importlib
import os
import pathlib
import signal
import subprocess
import sys
import time
from contextlib import ExitStack

HERE = pathlib.Path(__file__).resolve()
ROOT = (
    HERE.parents[2]
    if HERE.parent.name == "input-review" and HERE.parent.parent.name == "target"
    else HERE.parents[1]
)
ARTIFACTS = (ROOT / "target/input-review").resolve()


def artifact_path(value):
    path = (ROOT / value).resolve()
    if not path.is_relative_to(ARTIFACTS):
        raise ValueError("screenshots and logs must stay under target/input-review")
    return path


class XTest:
    def __init__(self, display):
        self.x11 = ctypes.CDLL("libX11.so.6")
        self.xtst = ctypes.CDLL("libXtst.so.6")
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
        self.x11.XDefaultRootWindow.restype = ctypes.c_ulong
        self.x11.XQueryTree.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.c_ulong),
            ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)),
            ctypes.POINTER(ctypes.c_uint),
        ]
        self.x11.XFetchName.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.POINTER(ctypes.c_void_p),
        ]
        self.x11.XFree.argtypes = [ctypes.c_void_p]
        self.x11.XMapRaised.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XSetInputFocus.argtypes = [
            ctypes.c_void_p,
            ctypes.c_ulong,
            ctypes.c_int,
            ctypes.c_ulong,
        ]
        self.x11.XFlush.argtypes = [ctypes.c_void_p]
        self.x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XKeysymToKeycode.restype = ctypes.c_uint
        self.x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self.x11.XStringToKeysym.restype = ctypes.c_ulong
        self.xtst.XTestFakeKeyEvent.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint,
            ctypes.c_int,
            ctypes.c_ulong,
        ]
        self.xtst.XTestFakeButtonEvent.argtypes = [
            ctypes.c_void_p,
            ctypes.c_uint,
            ctypes.c_int,
            ctypes.c_ulong,
        ]
        self.xtst.XTestFakeMotionEvent.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_int,
            ctypes.c_ulong,
        ]
        self.display = self.x11.XOpenDisplay(display.encode())
        if not self.display:
            raise RuntimeError(f"cannot open X display {display}")

    def windows(self):
        root = self.x11.XDefaultRootWindow(self.display)
        root_out, parent, children, count = (
            ctypes.c_ulong(),
            ctypes.c_ulong(),
            ctypes.POINTER(ctypes.c_ulong)(),
            ctypes.c_uint(),
        )
        self.x11.XQueryTree(
            self.display,
            root,
            ctypes.byref(root_out),
            ctypes.byref(parent),
            ctypes.byref(children),
            ctypes.byref(count),
        )
        result = []
        for i in range(count.value):
            name = ctypes.c_void_p()
            if (
                self.x11.XFetchName(self.display, children[i], ctypes.byref(name))
                and name.value
            ):
                title = ctypes.string_at(name.value).decode(errors="replace")
                self.x11.XFree(name)
                result.append((children[i], title))
        if children:
            self.x11.XFree(children)
        return result

    def focus(self, pattern):
        candidates = [
            (wid, title)
            for wid, title in self.windows()
            if pattern.lower() in title.lower()
        ]
        if not candidates:
            raise RuntimeError(
                f"no top-level window title contains {pattern!r}; windows={self.windows()}"
            )
        wid, title = candidates[-1]
        self.x11.XMapRaised(self.display, wid)
        self.x11.XSetInputFocus(self.display, wid, 2, 0)
        self.x11.XFlush(self.display)
        self.focused_window = wid
        print(f"focused 0x{wid:x}: {title}")

    def key(self, name, down):
        sym = self.x11.XStringToKeysym(name.encode())
        code = self.x11.XKeysymToKeycode(self.display, sym)
        if not code or not self.xtst.XTestFakeKeyEvent(
            self.display, code, int(down), 0
        ):
            raise RuntimeError(f"XTest key failed: {name}")
        self.x11.XFlush(self.display)

    def press(self, name, duration=0.15):
        self.key(name, True)
        time.sleep(duration)
        self.key(name, False)

    def button(self, button, down):
        if not self.xtst.XTestFakeButtonEvent(self.display, button, int(down), 0):
            raise RuntimeError(f"XTest button failed: {button}")
        self.x11.XFlush(self.display)

    def click(self, button=1):
        self.button(button, True)
        time.sleep(0.08)
        self.button(button, False)

    def focus_out(self):
        root = self.x11.XDefaultRootWindow(self.display)
        self.x11.XSetInputFocus(self.display, root, 2, 0)
        self.x11.XFlush(self.display)

    def focus_in(self):
        wid = getattr(self, "focused_window", None)
        if wid is None:
            raise RuntimeError("use --window before focus:in")
        self.x11.XMapRaised(self.display, wid)
        self.x11.XSetInputFocus(self.display, wid, 2, 0)
        self.x11.XFlush(self.display)

    def type_text(self, text):
        for char in text:
            sym = self.x11.XStringToKeysym(char.encode())
            code = self.x11.XKeysymToKeycode(self.display, sym)
            if not code:
                raise RuntimeError(f"no X keycode for {char!r}")
            self.xtst.XTestFakeKeyEvent(self.display, code, 1, 0)
            self.xtst.XTestFakeKeyEvent(self.display, code, 0, 0)
        self.x11.XFlush(self.display)

    def action(self, value):
        parts = value.split(":")
        if parts[0] == "move" and len(parts) == 3:
            self.move(int(parts[1]), int(parts[2]))
        elif parts[0] == "drag" and len(parts) == 5:
            x1, y1, x2, y2 = map(int, parts[1:])
            self.move(x1, y1)
            self.button(1, True)
            time.sleep(0.1)
            self.move(x2, y2)
            time.sleep(0.1)
            self.button(1, False)
        elif parts[0] == "click" and len(parts) in (3, 4):
            self.move(int(parts[1]), int(parts[2]))
            self.click(int(parts[3]) if len(parts) == 4 else 1)
        elif parts[0] == "key" and len(parts) == 2:
            self.press(parts[1])
        elif parts[0] == "key" and len(parts) == 3 and parts[2] in ("down", "up"):
            self.key(parts[1], parts[2] == "down")
        elif parts[0] == "mouse" and len(parts) == 3 and parts[2] in ("down", "up"):
            self.button(int(parts[1]), parts[2] == "down")
        elif parts[0] == "type" and len(parts) == 2:
            self.type_text(parts[1])
        elif (
            parts[0] == "wheel"
            and len(parts) == 2
            and parts[1].upper() in ("UP", "DOWN")
        ):
            self.click(4 if parts[1].upper() == "UP" else 5)
        elif parts[0] == "focus" and len(parts) == 2 and parts[1] == "out":
            self.focus_out()
        elif parts[0] == "focus" and len(parts) == 2 and parts[1] == "in":
            self.focus_in()
        elif parts[0] == "wait" and len(parts) == 2:
            duration = float(parts[1])
            if not 0 <= duration <= 5:
                raise ValueError("wait action must be between 0 and 5 seconds")
            time.sleep(duration)
        else:
            raise ValueError(f"invalid action: {value!r}")

    def move(self, x, y):
        if not self.xtst.XTestFakeMotionEvent(self.display, -1, x, y, 0):
            raise RuntimeError(f"XTest motion failed at {x},{y}")
        self.x11.XFlush(self.display)

    def close(self):
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.x11.XCloseDisplay(self.display)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--display", default=":99")
    parser.add_argument(
        "--window", help="focus the top-level window with this title fragment"
    )
    parser.add_argument(
        "--wait", type=float, default=4, help="viewer startup wait in seconds (max 30)"
    )
    parser.add_argument(
        "--profile", default="default", help="isolated config profile name"
    )
    parser.add_argument("--output", default="target/input-review/before.png")
    parser.add_argument("--viewer", default="target/debug/rocketsim-viewer")
    parser.add_argument("--port", default="39201")
    parser.add_argument(
        "--action",
        action="append",
        default=[],
        help="actions: click:X:Y[:button], move:X:Y, drag:X1:Y1:X2:Y2, key:KEY[:down|up], mouse:N:down|up, wheel:UP|DOWN, focus:out|in, pad:BTN_SOUTH[:0|1], axis:ABS_X:VALUE, wait:SECONDS, capture:FILE",
    )
    parser.add_argument(
        "--controller",
        action="store_true",
        help="create a temporary Xbox 360-style evdev gamepad",
    )
    parser.add_argument(
        "--drive",
        action="store_true",
        help="launch the built rocketsim_vis drive example",
    )
    parser.add_argument(
        "--control-probe",
        action="store_true",
        help="launch the sim-side control/state JSON probe",
    )
    parser.add_argument(
        "--probe-seconds",
        type=int,
        default=40,
        help="control-probe lifetime (1..120 seconds)",
    )
    parser.add_argument(
        "--control-log",
        help="sim-side JSON Lines path (default: target/input-review/<profile>-controls.jsonl)",
    )
    parser.add_argument("--mode", choices=("soccar", "thevoid"), default="soccar")

    parser.add_argument(
        "--settings", help="copy a TOML settings file into this isolated profile"
    )

    args = parser.parse_args()
    if not 0 <= args.wait <= 30:
        parser.error("--wait must be between 0 and 30 seconds")
    if not 1 <= args.probe_seconds <= 120:
        parser.error("--probe-seconds must be between 1 and 120")
    if len(args.action) > 300:
        parser.error("at most 300 actions are allowed per run")
    try:
        explicit_wait = sum(
            float(action.split(":", 1)[1])
            for action in args.action
            if action.startswith("wait:")
        )
    except ValueError:
        parser.error("wait actions need a numeric duration")
    if explicit_wait > 30:
        parser.error("total explicit wait time must not exceed 30 seconds")

    if args.drive and args.control_probe:
        parser.error("choose either --drive or --control-probe, not both")
    if not args.profile or any(
        ch not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_"
        for ch in args.profile
    ):
        parser.error(
            "profile must contain only letters, digits, hyphens, and underscores"
        )
    if args.control_log is None:
        args.control_log = f"target/input-review/{args.profile}-controls.jsonl"

    try:
        out = artifact_path(args.output)
    except ValueError as error:
        parser.error(str(error))
    out.parent.mkdir(parents=True, exist_ok=True)
    host_env = os.environ.copy()
    host_env.pop("WAYLAND_DISPLAY", None)
    xenv = host_env.copy()
    xenv["DISPLAY"] = args.display
    profile_config = ROOT / "target/input-review/config" / args.profile
    profile_config.mkdir(parents=True, exist_ok=True)
    xenv["XDG_CONFIG_HOME"] = str(profile_config.resolve())
    xenv["XDG_CACHE_HOME"] = str((ROOT / "target/input-review/cache").resolve())
    xenv["XDG_DATA_HOME"] = str((ROOT / "target/input-review/data").resolve())
    settings_path = profile_config / "rocketsim-vis/settings.toml"
    if args.settings:
        source_settings = pathlib.Path(args.settings).resolve()
        if not source_settings.is_file():
            parser.error(f"settings file does not exist: {source_settings}")
        settings_path.parent.mkdir(parents=True, exist_ok=True)
        settings_path.write_bytes(source_settings.read_bytes())

    occupied = subprocess.run(
        ["xwininfo", "-display", args.display, "-root"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if occupied.returncode == 0:
        raise RuntimeError(
            f"refusing to reuse existing X display {args.display}; choose an unused Xephyr display"
        )
    server = None
    viewer = None
    scene = None
    pad = None
    ecodes = None
    control_probe = None
    log_files = ExitStack()
    try:
        xephyr_log = log_files.enter_context(
            (ROOT / "target/input-review/xephyr.log").open("ab")
        )
        server = subprocess.Popen(
            ["Xephyr", args.display, "-screen", "1280x720", "-nolisten", "tcp"],
            stdout=subprocess.DEVNULL,
            stderr=xephyr_log,
            env=host_env,
        )
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            probe = subprocess.run(
                ["xwininfo", "-display", args.display, "-root"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
            )
            if probe.returncode == 0:
                break
            if server.poll() is not None:
                raise RuntimeError(f"Xephyr exited with status {server.returncode}")
            time.sleep(0.1)
        else:
            raise RuntimeError("Xephyr did not become ready within 8 seconds")

        if args.controller:
            try:
                evdev = importlib.import_module("evdev")
            except ModuleNotFoundError as error:
                if error.name != "evdev":
                    raise
                raise RuntimeError(
                    "--controller requires the Python 'evdev' package"
                ) from error
            AbsInfo, UInput, ecodes = evdev.AbsInfo, evdev.UInput, evdev.ecodes

            axes = {
                ecodes.ABS_X: AbsInfo(0, -32768, 32767, 16, 128, 0),
                ecodes.ABS_Y: AbsInfo(0, -32768, 32767, 16, 128, 0),
                ecodes.ABS_RX: AbsInfo(0, -32768, 32767, 16, 128, 0),
                ecodes.ABS_RY: AbsInfo(0, -32768, 32767, 16, 128, 0),
                ecodes.ABS_Z: AbsInfo(0, 0, 255, 0, 0, 0),
                ecodes.ABS_RZ: AbsInfo(0, 0, 255, 0, 0, 0),
                ecodes.ABS_HAT0X: AbsInfo(0, -1, 1, 0, 0, 0),
                ecodes.ABS_HAT0Y: AbsInfo(0, -1, 1, 0, 0, 0),
            }
            buttons = [
                ecodes.BTN_SOUTH,
                ecodes.BTN_EAST,
                ecodes.BTN_NORTH,
                ecodes.BTN_WEST,
                ecodes.BTN_TL,
                ecodes.BTN_TR,
                ecodes.BTN_SELECT,
                ecodes.BTN_START,
                ecodes.BTN_MODE,
                ecodes.BTN_THUMBL,
                ecodes.BTN_THUMBR,
            ]
            pad = UInput(
                {ecodes.EV_KEY: buttons, ecodes.EV_ABS: axes},
                name="Xbox 360 Controller",
                vendor=0x045E,
                product=0x028E,
                version=1,
                bustype=ecodes.BUS_USB,
            )
            print(
                f"created virtual pad {pad.device.path}: Xbox 360 Controller (045e:028e)"
            )
            time.sleep(0.75)

        viewer_log = log_files.enter_context(
            (ROOT / "target/input-review/viewer.log").open("ab")
        )
        viewer = subprocess.Popen(
            [str((ROOT / args.viewer).resolve()), args.port],
            env=xenv,
            stdout=viewer_log,
            stderr=subprocess.STDOUT,
        )
        if args.drive:
            scene_cmd = [
                str(ROOT / "target/debug/examples/drive"),
                "--viewer",
                f"127.0.0.1:{args.port}",
            ]
            scene_log = log_files.enter_context(
                (ROOT / "target/input-review/scene.log").open("ab")
            )
            scene = subprocess.Popen(
                scene_cmd, env=xenv, stdout=scene_log, stderr=subprocess.STDOUT
            )
        if args.control_probe:
            probe_bin = (
                ROOT
                / "target/input-review/control-probe/target/debug/input-control-probe"
            )
            if not probe_bin.is_file():
                raise RuntimeError(
                    "build the sim probe after the viewer: cargo build --offline --manifest-path tools/Cargo.toml --target-dir target/input-review/control-probe/target"
                )
            try:
                probe_log = artifact_path(args.control_log)
            except ValueError as error:
                raise RuntimeError(str(error)) from error
            probe_log.parent.mkdir(parents=True, exist_ok=True)
            probe_file = log_files.enter_context(probe_log.open("wb"))
            control_probe = subprocess.Popen(
                [
                    str(probe_bin),
                    f"127.0.0.1:{args.port}",
                    args.mode,
                    str(args.probe_seconds),
                ],
                env=xenv,
                stdout=probe_file,
                stderr=subprocess.STDOUT,
            )

        time.sleep(args.wait)
        if viewer.poll() is not None:
            raise RuntimeError(
                f"viewer exited with status {viewer.returncode}; see target/input-review/viewer.log"
            )
        display = XTest(args.display)
        try:
            windows = display.windows()
            print("windows:", windows)
            if args.window:
                display.focus(args.window)
                time.sleep(0.5)

            for action in args.action:
                if action.startswith("capture:"):
                    try:
                        capture = artifact_path(action[len("capture:") :])
                    except ValueError as error:
                        raise RuntimeError(str(error)) from error
                    capture.parent.mkdir(parents=True, exist_ok=True)
                    xwd_path = capture.with_suffix(".xwd")
                elif action.startswith("pad:"):
                    if pad is None or ecodes is None:
                        raise RuntimeError("pad action requires --controller")
                    fields = action.split(":")
                    code = getattr(ecodes, fields[1])
                    value = int(fields[2]) if len(fields) == 3 else 1
                    pad.write(ecodes.EV_KEY, code, value)
                    pad.syn()
                    time.sleep(0.2)
                    continue
                elif action.startswith("axis:"):
                    if pad is None or ecodes is None:
                        raise RuntimeError("axis action requires --controller")
                    _, axis_name, axis_value = action.split(":", 2)
                    pad.write(
                        ecodes.EV_ABS, getattr(ecodes, axis_name), int(axis_value)
                    )
                    pad.syn()
                    time.sleep(0.2)
                    continue
                else:
                    display.action(action)
                    time.sleep(0.15)
                    continue
                subprocess.run(
                    ["xwd", "-display", args.display, "-root", "-out", str(xwd_path)],
                    check=True,
                )
                subprocess.run(
                    [
                        sys.executable,
                        str(ROOT / "tools/xwd2png.py"),
                        str(xwd_path),
                        str(capture),
                    ],
                    check=True,
                )
                xwd_path.unlink(missing_ok=True)
                print(f"captured {capture}")

            subprocess.run(
                [
                    "xwd",
                    "-display",
                    args.display,
                    "-root",
                    "-out",
                    str(out.with_suffix(".xwd")),
                ],
                check=True,
            )
            subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "tools/xwd2png.py"),
                    str(out.with_suffix(".xwd")),
                    str(out),
                ],
                check=True,
            )
            out.with_suffix(".xwd").unlink(missing_ok=True)
            print(f"captured {out}")
        finally:
            display.close()
    finally:
        try:
            if pad is not None:
                pad.close()
            for proc in (control_probe, scene, viewer, server):
                if proc is not None and proc.poll() is None:
                    proc.send_signal(signal.SIGTERM)
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=2)
        finally:
            log_files.close()


if __name__ == "__main__":
    main()
