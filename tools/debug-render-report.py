#!/usr/bin/env python3
"""vis-v2 debug-render report: list captures, logs, metrics."""

import argparse
import json
import pathlib

HERE = pathlib.Path(__file__).resolve()
ROOT = HERE.parents[1]
SCENARIOS = ("wire", "solid", "showcase", "stress")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default="target/input-review/debug-render")
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    out_dir = (ROOT / args.out_dir).resolve()
    lines = ["vis-v2 debug-render report", f"dir: {out_dir}", ""]
    for name in SCENARIOS:
        work = out_dir / name
        lines.append(f"## {name}")
        if not work.is_dir():
            lines.append("missing (not run)")
            continue
        shot = work / f"{name}.png"
        if shot.is_file():
            lines.append(f"shot: {shot.name} ({shot.stat().st_size} bytes)")
            lines.append(f"inspect with read_file: {shot}")
        else:
            lines.append("shot: missing")
        for log_name in ("viewer.log", "demo.log"):
            item = work / log_name
            if item.is_file():
                lines.append(f"log: {log_name} ({item.stat().st_size} bytes)")
            else:
                lines.append(f"log: {log_name} missing")
        metrics = work / "metrics.json"
        if metrics.is_file():
            try:
                data = json.loads(metrics.read_text())
            except (ValueError, OSError) as error:
                lines.append(f"metrics: unreadable ({error})")
                continue
            if isinstance(data, dict):
                keys = ", ".join(sorted(str(k) for k in data.keys())[:12])
                lines.append(f"metrics: keys [{keys}]")
                for key in sorted(data.keys())[:12]:
                    lines.append(f"  {key} = {data[key]}")
            else:
                lines.append("metrics: non-object JSON")
        else:
            lines.append("metrics: missing (needs viewer --debug-stats)")
        lines.append("")
    text = "\n".join(lines).rstrip() + "\n"
    if args.output:
        dest = (ROOT / args.output).resolve()
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(text)
        print(f"vis-v2: wrote {dest}")
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
