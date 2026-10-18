#!/usr/bin/env python3
"""Prepare the pinned RLGym collision mesh fixture directory."""

from __future__ import annotations

import argparse
import os
import shutil
import tempfile
from pathlib import Path

EXPECTED_COUNTS = {"soccar": 16, "hoops": 12, "dropshot": 2}


def selected_meshes(source: Path) -> dict[str, list[Path]]:
    if source.is_symlink() or not source.is_dir():
        raise ValueError(f"Source must be a real directory: {source}")
    result: dict[str, list[Path]] = {}
    for mode, count in EXPECTED_COUNTS.items():
        folder = source / mode
        if folder.is_symlink() or not folder.is_dir():
            raise ValueError(f"Missing or invalid mesh directory: {folder}")
        files = sorted(folder.glob("*.cmf"))
        if len(files) != count:
            raise ValueError(f"Expected {count} {mode} .cmf files, found {len(files)}")
        if any(path.is_symlink() or not path.is_file() for path in files):
            raise ValueError(f"All {mode} meshes must be regular files")
        result[mode] = files
    return result


def prepare(source: Path, destination: Path) -> None:
    meshes = selected_meshes(source)
    if destination.is_symlink():
        raise ValueError(f"Destination must not be a symlink: {destination}")
    if destination.exists():
        if not destination.is_dir():
            raise ValueError(f"Destination is not a directory: {destination}")
        expected = {f"{mode}/{path.name}" for mode, files in meshes.items() for path in files}
        actual = {p.relative_to(destination).as_posix() for p in destination.rglob("*") if p.is_file()}
        if actual != expected or any(p.is_symlink() for p in destination.rglob("*")):
            raise ValueError(f"Existing destination does not match the fixture: {destination}")
        for mode, files in meshes.items():
            for source_file in files:
                if (destination / mode / source_file.name).read_bytes() != source_file.read_bytes():
                    raise ValueError(f"Existing mesh differs: {mode}/{source_file.name}")
        return

    destination.parent.mkdir(parents=True, exist_ok=True)
    staging = Path(tempfile.mkdtemp(prefix=f".{destination.name}.", dir=destination.parent))
    try:
        for mode, files in meshes.items():
            target_folder = staging / mode
            target_folder.mkdir()
            for source_file in files:
                shutil.copyfile(source_file, target_folder / source_file.name, follow_symlinks=False)
        os.rename(staging, destination)
    except BaseException:
        shutil.rmtree(staging, ignore_errors=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="RLGym collision_meshes source folder")
    parser.add_argument("--destination", type=Path, default=Path("collision_meshes"))
    args = parser.parse_args()
    try:
        prepare(args.source, args.destination)
    except (OSError, ValueError) as error:
        parser.error(str(error))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
