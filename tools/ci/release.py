#!/usr/bin/env python3
"""Build and verify RocketSim release archives."""

from __future__ import annotations

import argparse
import hashlib
import subprocess
import tarfile
import tomllib
import zipfile
from pathlib import Path, PurePosixPath

TARGETS = (
    "x86_64-unknown-linux-gnu",
    "x86_64-pc-windows-msvc",
    "x86_64-apple-darwin",
    "aarch64-apple-darwin",
)
BINARIES = ("rocketsim-viewer",)
ARCHIVE_FILES = ("LICENSE", "LICENSES.md", "README.md", "MODEL_LICENSE_README.md")
PKG_URL = "{ repo }/releases/download/v{ version }/{ name }-{ version }-{ target }{ archive-suffix }"


class ReleaseError(Exception):
    """A release input or artifact is invalid."""


def _load_toml(path: Path) -> dict:
    try:
        with path.open("rb") as stream:
            return tomllib.load(stream)
    except (OSError, tomllib.TOMLDecodeError) as exc:
        raise ReleaseError(f"Cannot read {path}: {exc}") from exc


def _manifest_version(path: Path, version: str) -> dict:
    package = _load_toml(path).get("package", {})
    value = package.get("version")
    if value not in (version, {"workspace": True}):
        raise ReleaseError(f"Package version does not match workspace version: {path}")
    return package


def package_version(root: Path) -> str:
    workspace = _load_toml(root / "Cargo.toml").get("workspace", {}).get("package", {})
    version = workspace.get("version")
    if not isinstance(version, str) or not version:
        raise ReleaseError("Workspace package version is missing")
    _manifest_version(root / "rocketsim_viewer" / "Cargo.toml", version)
    _manifest_version(root / "rocketsim" / "Cargo.toml", version)
    _manifest_version(root / "rocketsim_vis" / "Cargo.toml", version)
    return version


def _dependency_version(manifest: dict, name: str) -> str | None:
    dependencies = manifest.get("dependencies", {})
    value = dependencies.get(name)
    if isinstance(value, str):
        return value
    if isinstance(value, dict):
        return value.get("version")
    return None


def _check_dependency(manifest: dict, crate: str, version: str, path: Path) -> None:
    if _dependency_version(manifest, crate) != version:
        raise ReleaseError(
            f"Dependency {crate} version does not match workspace version in {path}"
        )


def _binary_names(target: str) -> tuple[str, ...]:
    suffix = ".exe" if target == "x86_64-pc-windows-msvc" else ""
    return tuple(name + suffix for name in BINARIES)


def _archive_name(version: str, target: str, fmt: str) -> str:
    suffix = "zip" if fmt == "zip" else "tgz"
    return f"rocketsim_viewer-{version}-{target}.{suffix}"


def _expected_members(target: str) -> set[str]:
    return set(_binary_names(target) + ARCHIVE_FILES)


def check(root: Path, tag: str | None = None) -> str:
    version = package_version(root)
    if tag is not None and tag != f"v{version}":
        raise ReleaseError(f"Tag must be v{version}, got {tag}")
    viewer_path = root / "rocketsim_viewer" / "Cargo.toml"
    viewer_manifest = _load_toml(viewer_path)
    viewer = viewer_manifest.get("package", {})
    if viewer.get("autobins") is not False:
        raise ReleaseError("Viewer package must set autobins = false")
    features = viewer_manifest.get("features", {})
    default_features = set()
    pending = ["default"] if "default" in features else []
    while pending:
        feature = pending.pop()
        if feature not in default_features:
            default_features.add(feature)
            pending.extend(features.get(feature, []))
    default_bins = {
        item.get("name")
        for item in viewer_manifest.get("bin", [])
        if set(item.get("required-features", [])).issubset(default_features)
    }
    if default_bins != set(BINARIES):
        raise ReleaseError("Viewer default binaries must be exactly: rocketsim-viewer")
    vis_path = root / "rocketsim_vis" / "Cargo.toml"
    vis_manifest = _load_toml(vis_path)
    _check_dependency(vis_manifest, "rocketsim", version, vis_path)
    _check_dependency(viewer_manifest, "rocketsim", version, viewer_path)
    _check_dependency(viewer_manifest, "rocketsim_vis", version, viewer_path)

    metadata = viewer.get("metadata", {}).get("binstall", {})
    if metadata.get("pkg-url") != PKG_URL:
        raise ReleaseError("binstall pkg-url does not match the release archive name")
    if metadata.get("bin-dir") != "{ bin }{ binary-ext }":
        raise ReleaseError(
            "binstall bin-dir must install binaries from the archive root"
        )
    if metadata.get("pkg-fmt") != "tgz":
        raise ReleaseError("binstall pkg-fmt must be tgz")
    if metadata.get("disabled-strategies") != ["quick-install"]:
        raise ReleaseError("binstall must disable quick-install")
    overrides = metadata.get("overrides", {})
    if (
        not isinstance(overrides, dict)
        or overrides.get("x86_64-pc-windows-msvc", {}).get("pkg-fmt") != "zip"
    ):
        raise ReleaseError("binstall must use zip for x86_64-pc-windows-msvc")

    if not (root / "Cargo.lock").is_file():
        raise ReleaseError("Root Cargo.lock is missing")
    return version


def _safe_members(names: list[str]) -> set[str]:
    safe = set()
    for name in names:
        path = PurePosixPath(name)
        if (
            path.is_absolute()
            or ".." in path.parts
            or len(path.parts) != 1
            or name in ("", ".")
        ):
            raise ReleaseError(f"Unsafe archive path: {name}")
        if name in safe:
            raise ReleaseError(f"Duplicate archive member: {name}")
        safe.add(name)
    return safe


def _archive_members(path: Path) -> set[str]:
    try:
        if path.suffix == ".zip":
            with zipfile.ZipFile(path) as archive:
                infos = archive.infolist()
                for info in infos:
                    mode = info.external_attr >> 16
                    if info.is_dir() or (mode & 0o170000) == 0o120000:
                        raise ReleaseError(
                            f"Archive contains a directory or symlink: {info.filename}"
                        )
                return _safe_members([item.filename for item in infos])
        with tarfile.open(path, "r:gz") as archive:
            members = archive.getmembers()
            for member in members:
                if not member.isfile():
                    raise ReleaseError(
                        f"Archive contains a non-file member: {member.name}"
                    )
            return _safe_members([item.name for item in members])
    except (OSError, tarfile.TarError, zipfile.BadZipFile) as exc:
        raise ReleaseError(f"Invalid archive {path}: {exc}") from exc


def _digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _check_archive(path: Path, target: str, version: str) -> None:
    fmt = "zip" if target == "x86_64-pc-windows-msvc" else "tgz"
    expected_name = _archive_name(version, target, fmt)
    if path.name != expected_name:
        raise ReleaseError(f"Expected archive {expected_name}, got {path.name}")
    if _archive_members(path) != _expected_members(target):
        raise ReleaseError(f"Archive has an incorrect member set: {path.name}")
    checksum = path.with_name(path.name + ".sha256")
    if checksum.is_symlink() or not checksum.is_file():
        raise ReleaseError(f"Missing regular checksum: {checksum.name}")
    expected = f"{_digest(path)}  {path.name}\n"
    try:
        actual = checksum.read_text(encoding="ascii")
    except (OSError, UnicodeError) as exc:
        raise ReleaseError(f"Checksum is invalid: {checksum.name}") from exc
    if actual != expected:
        raise ReleaseError(f"Checksum is invalid: {checksum.name}")


def _smoke_test(viewer: Path, root: Path, version: str, timeout: int) -> None:
    try:
        result = subprocess.run(
            [str(viewer), "--version"],
            cwd=root,
            check=True,
            timeout=timeout,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise ReleaseError(f"Version smoke test failed: {exc}") from exc
    expected = f"rocketsim-viewer {version}\n"
    if result.stdout != expected:
        raise ReleaseError(
            f"Version smoke test returned {result.stdout!r}, expected {expected!r}"
        )
    try:
        subprocess.run(
            [str(viewer), "--help"],
            cwd=root,
            check=True,
            timeout=timeout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise ReleaseError(f"Help smoke test failed: {exc}") from exc


def _archive_sources(
    root: Path, target: str, binaries: list[Path]
) -> list[tuple[Path, str]]:
    sources = list(zip(binaries, _binary_names(target))) + [
        (root / "LICENSE", "LICENSE"),
        (root / "rocketsim_viewer" / "LICENSES.md", "LICENSES.md"),
        (root / "rocketsim_viewer" / "README.md", "README.md"),
        (
            root / "rocketsim_viewer" / "src" / "models" / "MODEL_LICENSE_README.md",
            "MODEL_LICENSE_README.md",
        ),
    ]
    for source, _ in sources:
        if source.is_symlink() or not source.is_file():
            raise ReleaseError(f"Required regular archive input is missing: {source}")
    return sources


def package(root: Path, target: str, timeout: int = 20) -> Path:
    if target not in TARGETS:
        raise ReleaseError(f"Unsupported target: {target}")
    version = check(root)
    inputs = root / "target" / target / "dist"
    binaries = [inputs / name for name in _binary_names(target)]
    for binary in binaries:
        if binary.is_symlink() or not binary.is_file():
            raise ReleaseError(f"Missing regular binary: {binary}")
    sources = _archive_sources(root, target, binaries)
    _smoke_test(binaries[0], root, version, timeout)
    fmt = "zip" if target == "x86_64-pc-windows-msvc" else "tgz"
    output = root / "dist"
    output.mkdir(parents=True, exist_ok=True)
    archive_path = output / _archive_name(version, target, fmt)
    if fmt == "zip":
        with zipfile.ZipFile(
            archive_path, "w", compression=zipfile.ZIP_DEFLATED
        ) as archive:
            for source, member in sources:
                archive.write(source, member)
    else:
        with tarfile.open(archive_path, "w:gz") as archive:
            for source, member in sources:
                archive.add(source, arcname=member, recursive=False)
    checksum = output / (archive_path.name + ".sha256")
    checksum.write_text(
        f"{_digest(archive_path)}  {archive_path.name}\n", encoding="ascii"
    )
    _check_archive(archive_path, target, version)
    return archive_path


def verify(root: Path, tag: str) -> Path:
    version = check(root, tag)
    dist = root / "dist"
    expected_assets = set()
    archives = []
    for target in TARGETS:
        fmt = "zip" if target == "x86_64-pc-windows-msvc" else "tgz"
        name = _archive_name(version, target, fmt)
        archive = dist / name
        expected_assets.update((name, name + ".sha256"))
        if archive.is_symlink() or not archive.is_file():
            raise ReleaseError(f"Missing regular archive: {archive.name}")
        _check_archive(archive, target, version)
        archives.append(archive)
    expected_assets.add("SHA256SUMS")
    try:
        actual_assets = {item.name for item in dist.iterdir()}
    except OSError as exc:
        raise ReleaseError(f"Cannot read release asset directory: {exc}") from exc
    if actual_assets - expected_assets:
        raise ReleaseError(
            f"Unexpected release assets: {sorted(actual_assets - expected_assets)}"
        )
    for item in dist.iterdir():
        if item.is_symlink() or not item.is_file():
            raise ReleaseError(f"Release asset is not a regular file: {item.name}")
    sums = dist / "SHA256SUMS"
    sums.write_text(
        "".join(f"{_digest(path)}  {path.name}\n" for path in archives),
        encoding="ascii",
    )
    return sums


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    check_parser = commands.add_parser("check")
    check_parser.add_argument("--tag")
    package_parser = commands.add_parser("package")
    package_parser.add_argument("--target", required=True, choices=TARGETS)
    verify_parser = commands.add_parser("verify")
    verify_parser.add_argument("--tag", required=True)
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[2]
    try:
        if args.command == "check":
            print(f"Release metadata is valid for {check(root, args.tag)}")
        elif args.command == "package":
            print(package(root, args.target))
        else:
            print(verify(root, args.tag))
    except ReleaseError as exc:
        parser.exit(1, f"release.py: error: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
