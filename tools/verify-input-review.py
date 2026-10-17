#!/usr/bin/env python3
"""Assert observed sim-side input records from target/input-review/controls.jsonl."""

import argparse
import json
import math
import operator
import pathlib
import sys

OPS = {"=": operator.eq, "==": operator.eq, "!=": operator.ne,
       ">": operator.gt, ">=": operator.ge, "<": operator.lt, "<=": operator.le}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", nargs="?", default="target/input-review/controls.jsonl")
    parser.add_argument("--require", action="append", default=[], metavar="FIELD<OP>VALUE",
                        help="require at least one record matching a dotted field comparison")
    parser.add_argument("--transition", action="append", default=[], metavar="FIELD",
                        help="require the field to have both neutral and non-neutral values")
    parser.add_argument("--action-bit", action="append", default=[], type=int, metavar="N",
                        help="require the simulation to consume action bit N")
    parser.add_argument("--button-bit", action="append", default=[], type=int, metavar="N",
                        help="require a ControlMsg button bit N")
    parser.add_argument("--mutator-bit", action="append", default=[], type=int, metavar="N",
                        help="require a ControlMsg mutator bit N")
    parser.add_argument("--neutral", action="store_true", help="require a neutral input record, such as after focus loss")
    parser.add_argument("--state-moved", action="store_true", help="require car position to change by more than 1 unit")
    args = parser.parse_args()

    path = pathlib.Path(args.log)
    if not path.is_file():
        parser.error(f"record log does not exist: {path}")
    records = []
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        try:
            record = json.loads(line)
        except json.JSONDecodeError as error:
            parser.error(f"invalid JSON at {path}:{line_number}: {error}")
        records.append(record)
    failures = []
    if not any(r.get("kind") == "control" for r in records):
        failures.append("no control records received from the viewer")

    def value_at(record, field):
        value = record
        for key in field.split("."):
            if not isinstance(value, dict) or key not in value:
                return None
            value = value[key]
        return value

    import re
    pattern = re.compile(r"^([A-Za-z0-9_.]+)(>=|<=|==|!=|=|>|<)(.+)$")
    for requirement in args.require:
        match = pattern.match(requirement)
        if not match:
            parser.error(f"invalid requirement: {requirement!r}")
        field, op, expected_text = match.groups()
        try:
            expected = json.loads(expected_text)
        except json.JSONDecodeError:
            expected = expected_text
        matched = False
        for record in records:
            actual = value_at(record, field)
            if actual is None:
                continue
            try:
                if OPS[op](actual, expected):
                    matched = True
                    break
            except TypeError:
                continue
        if not matched:
            failures.append(f"no record satisfies {requirement}")

    for field in args.transition:
        values = [value_at(r, field) for r in records if value_at(r, field) is not None]
        numeric = [float(v) for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
        if not any(abs(value) < 0.1 for value in numeric) or not any(abs(value) > 0.8 for value in numeric):
            failures.append(f"{field} did not transition between neutral and active values")

    for bit in args.action_bit:
        if bit < 0 or bit > 7:
            parser.error("action bit must be between 0 and 7")
        if not any(int(r.get("actions", 0)) & (1 << bit) for r in records):
            failures.append(f"simulation did not consume action bit {bit}")

    for option, field in ((args.button_bit, "buttons"), (args.mutator_bit, "mutators")):
        for bit in option:
            if bit < 0 or bit > 7:
                parser.error(f"{field} bit must be between 0 and 7")
            if not any(int(r.get(field, 0)) & (1 << bit) for r in records):
                failures.append(f"no {field} record contains bit {bit}")

    if args.neutral and not any(
        all(not r.get(field, False) for field in ("jump", "boost", "handbrake"))
        and all(abs(float(r.get(field, 0))) < 0.05 for field in ("throttle", "steer", "pitch", "yaw", "roll"))
        for r in records if r.get("kind") in ("control", "neutral")
    ):
        failures.append("no neutral control record was observed")

    if args.state_moved:
        states = [r for r in records if r.get("kind") == "state" and all(k in r for k in ("x", "y", "z"))]
        if len(states) < 2:
            failures.append("fewer than two simulation state records were observed")
        elif max(math.dist((r["x"], r["y"], r["z"]), (states[0]["x"], states[0]["y"], states[0]["z"])) for r in states[1:]) <= 1:
            failures.append("simulated car position did not change by more than 1 unit")

    if failures:
        print("INPUT REVIEW FAILED", file=sys.stderr)
        for failure in failures:
            print(f"- {failure}", file=sys.stderr)
        return 1
    print(f"INPUT REVIEW PASS: {len(records)} JSON records checked from {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
