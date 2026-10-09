#!/usr/bin/env python3
"""Compare a fresh screenshot against a checked-in reference.

Usage: tools/vis-diff.py <reference.png> <fresh.png> [--tol 8] [--budget 1.0]

Exits 0 if the share of pixels differing by more than `tol` (per channel,
0-255) is at most `budget` percent, else 1. Size mismatch is a failure.
"""
import sys

import numpy as np
from PIL import Image


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    tol = 8
    budget = 1.0
    for i, a in enumerate(sys.argv):
        if a == "--tol":
            tol = int(sys.argv[i + 1])
        elif a == "--budget":
            budget = float(sys.argv[i + 1])
    if len(args) != 2:
        print("usage: vis-diff.py <reference.png> <fresh.png>", file=sys.stderr)
        sys.exit(2)
    ref = np.asarray(Image.open(args[0]).convert("RGB")).astype(np.int16)
    fresh = np.asarray(Image.open(args[1]).convert("RGB")).astype(np.int16)
    if ref.shape != fresh.shape:
        print(f"SIZE MISMATCH {ref.shape} vs {fresh.shape}", file=sys.stderr)
        sys.exit(1)
    diff = np.abs(ref - fresh).max(axis=2)
    frac = (diff > tol).mean() * 100.0
    print(f"drift: {frac:.3f}% pixels differ by >{tol} (budget {budget:.2f}%)")
    sys.exit(0 if frac <= budget else 1)


if __name__ == "__main__":
    main()
