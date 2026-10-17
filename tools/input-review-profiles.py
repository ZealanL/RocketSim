#!/usr/bin/env python3
"""Write isolated TOML profiles for real viewer input-source checks."""

import argparse
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
UNBOUND = ("Unbound",)
COMMON = {
    "MenuToggle": ("Key", "Escape"),
    "Follow1": ("Key", "Num1"),
    "FreeCam": ("Key", "F"),
    "FreeForward": ("Key", "W"),
    "FreeBack": ("Key", "S"),
    "FreeLeft": ("Key", "A"),
    "FreeRight": ("Key", "D"),
    "FreeUp": ("Key", "Space"),
    "FreeDown": ("Key", "Shift"),
}
PROFILES = {
    "keyboard": [("ThrottleForward", ("Key", "ArrowUp")), ("ThrottleBack", ("Key", "ArrowDown"))],
    "mouse-button": [("Jump", ("Mouse", "Primary")), ("Boost", ("Mouse", "Secondary"))],
    "mouse-axis": [
        ("LookRight", ("MouseAxis", "X", 1)), ("LookLeft", ("MouseAxis", "X", -1)),
        ("LookUp", ("MouseAxis", "Y", 1)), ("LookDown", ("MouseAxis", "Y", -1)),
    ],
    "wheel": [("ThrottleForward", ("Wheel", "Y", 1)), ("ThrottleBack", ("Wheel", "Y", -1))],
    "pad-button": [("Jump", ("PadButton", "South")), ("Boost", ("PadButton", "East"))],
    "pad-axis": [("SteerRight", ("PadAxis", "LeftStickX", 1)), ("SteerLeft", ("PadAxis", "LeftStickX", -1))],
    "pad-trigger": [("FreeUp", ("PadAxis", "RightZ", 1)), ("FreeDown", ("PadAxis", "LeftZ", 1))],
    "pad-raw": [("ThrottleForward", ("PadRaw", 196608, 1)), ("ThrottleBack", ("PadRaw", 196608, -1))],
    "shared-key": [("ThrottleForward", ("Key", "ArrowUp")), ("Jump", ("Key", "ArrowUp"))],
}


def toml_source(source):
    kind, *args = source
    if kind in ("Key", "Mouse", "PadButton"):
        return f'{{ type = "{kind}", value = "{args[0]}" }}'
    if kind in ("MouseAxis", "Wheel", "PadAxis"):
        return f'{{ type = "{kind}", value = {{ axis = "{args[0]}", sign = {args[1]} }} }}'
    if kind == "PadRaw":
        return f'{{ type = "PadRaw", value = {{ code = {args[0]}, sign = {args[1]} }} }}'
    if kind == "Unbound":
        return '{ type = "Unbound" }'
    raise ValueError(f"unknown source {source!r}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="target/input-review/profiles")
    args = parser.parse_args()
    out_dir = (ROOT / args.output_dir).resolve()
    target_root = (ROOT / "target/input-review").resolve()
    if not out_dir.is_relative_to(target_root):
        parser.error("output directory must stay under target/input-review")
    out_dir.mkdir(parents=True, exist_ok=True)

    for name, action_sources in PROFILES.items():
        bindings = dict(COMMON)
        bindings.update(action_sources)
        lines = ["[action_bindings.bindings]"]
        for action_name, binding in bindings.items():
            lines.append(f'"{action_name}" = [{toml_source(binding)}, {toml_source(UNBOUND)}]')
        (out_dir / f"{name}.toml").write_text("\n".join(lines) + "\n")
    print(f"wrote {len(PROFILES)} profiles to {out_dir}")


if __name__ == "__main__":
    main()
