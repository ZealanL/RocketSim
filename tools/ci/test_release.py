import hashlib
import tarfile
import tempfile
import unittest
import zipfile
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import release


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self._fixture()

    def tearDown(self):
        self.temp.cleanup()

    def _fixture(self):
        (self.root / "Cargo.toml").write_text(
            '[workspace.package]\nversion = "0.3.0"\nrepository = "https://example.test/repo"\n',
            encoding="utf-8",
        )
        (self.root / "Cargo.lock").write_text("", encoding="utf-8")
        sim = self.root / "rocketsim"
        sim.mkdir()
        (sim / "Cargo.toml").write_text(
            '[package]\nname = "rocketsim"\nversion.workspace = true\n', encoding="utf-8"
        )
        vis = self.root / "rocketsim_vis"
        vis.mkdir()
        (vis / "Cargo.toml").write_text(
            '[package]\nname = "rocketsim_vis"\nversion.workspace = true\n'
            '[dependencies]\nrocketsim = { path = "../rocketsim", version = "0.3.0" }\n',
            encoding="utf-8",
        )
        viewer = self.root / "rocketsim_viewer"
        models = viewer / "src" / "models"
        models.mkdir(parents=True)
        (viewer / "README.md").write_text("viewer readme", encoding="utf-8")
        (models / "MODEL_LICENSE_README.md").write_text("model notice", encoding="utf-8")
        metadata = """[package]
name = "rocketsim_viewer"
version.workspace = true
autobins = false
repository.workspace = true
[[bin]]
name = "rocketsim-viewer"
path = "src/main.rs"
[[bin]]
name = "mesh_stats"
path = "src/bin/mesh_stats.rs"
[dependencies]
rocketsim = { path = "../rocketsim", version = "0.3.0" }
rocketsim_vis = { path = "../rocketsim_vis", version = "0.3.0" }
[package.metadata.binstall]
pkg-url = "{ repo }/releases/download/v{ version }/{ name }-{ version }-{ target }{ archive-suffix }"
bin-dir = "{ bin }{ binary-ext }"
pkg-fmt = "tgz"
disabled-strategies = ["quick-install"]
[package.metadata.binstall.overrides.x86_64-pc-windows-msvc]
pkg-fmt = "zip"
"""
        (viewer / "Cargo.toml").write_text(metadata, encoding="utf-8")
        (self.root / "LICENSE").write_text("license", encoding="utf-8")

    def _binary_file_names(self, target):
        return release._binary_names(target)

    def _make_archive(self, target):
        windows = target == "x86_64-pc-windows-msvc"
        extension = "zip" if windows else "tgz"
        name = f"rocketsim_viewer-0.3.0-{target}.{extension}"
        dist = self.root / "dist"
        dist.mkdir(exist_ok=True)
        path = dist / name
        members = release._expected_members(target)
        if windows:
            with zipfile.ZipFile(path, "w") as archive:
                for member in members:
                    archive.writestr(member, b"test")
        else:
            with tarfile.open(path, "w:gz") as archive:
                for member in members:
                    data = self.root / member
                    data.write_bytes(b"test")
                    archive.add(data, arcname=member)
        path.with_name(name + ".sha256").write_text(
            f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {name}\n", encoding="ascii"
        )
        return path

    def test_check_versions_metadata_and_all_target_urls(self):
        self.assertEqual(release.check(self.root, "v0.3.0"), "0.3.0")
        self.assertEqual(len(release.TARGETS), 4)
        self.assertEqual(release._expected_members(release.TARGETS[1]), {
            "rocketsim-viewer.exe", "mesh_stats.exe", *release.ARCHIVE_FILES
        })

    def test_check_rejects_wrong_tag(self):
        with self.assertRaisesRegex(release.ReleaseError, "Tag must be"):
            release.check(self.root, "v9.9.9")

    def test_check_rejects_inconsistent_package_version(self):
        (self.root / "rocketsim" / "Cargo.toml").write_text(
            '[package]\nname = "rocketsim"\nversion = "1.0.0"\n', encoding="utf-8"
        )
        with self.assertRaisesRegex(release.ReleaseError, "does not match"):
            release.check(self.root)

    def test_check_rejects_dependency_version_mismatch(self):
        path = self.root / "rocketsim_vis" / "Cargo.toml"
        path.write_text(path.read_text(encoding="utf-8").replace('version = "0.3.0"', 'version = "1.0.0"'), encoding="utf-8")
        with self.assertRaisesRegex(release.ReleaseError, "Dependency rocketsim version"):
            release.check(self.root)

    def test_check_requires_both_declared_bins_and_disables_autobins(self):
        path = self.root / "rocketsim_viewer" / "Cargo.toml"
        original = path.read_text(encoding="utf-8")
        path.write_text(original.replace("autobins = false", "autobins = true"), encoding="utf-8")
        with self.assertRaisesRegex(release.ReleaseError, "autobins"):
            release.check(self.root)
        path.write_text(original.replace('name = "mesh_stats"', 'name = "other"'), encoding="utf-8")
        with self.assertRaisesRegex(release.ReleaseError, "exactly both"):
            release.check(self.root)

    @mock.patch.object(release.subprocess, "run")
    def test_package_creates_archives_for_all_targets(self, run):
        for target in release.TARGETS:
            inputs = self.root / "target" / target / "dist"
            inputs.mkdir(parents=True)
            for binary in self._binary_file_names(target):
                file = inputs / binary
                file.write_bytes(b"fake executable")
                file.chmod(0o755)
            run.side_effect = [
                SimpleNamespace(stdout="rocketsim-viewer 0.3.0\n"),
                SimpleNamespace(stdout=""),
            ]
            archive = release.package(self.root, target)
            self.assertEqual(archive.parent, self.root / "dist")
            self.assertTrue(archive.with_name(archive.name + ".sha256").is_file())
            self.assertEqual(
                release._archive_members(archive), release._expected_members(target)
            )
            if target == "x86_64-pc-windows-msvc":
                with zipfile.ZipFile(archive) as actual:
                    self.assertEqual(
                        set(actual.namelist()), release._expected_members(target)
                    )
                    self.assertEqual(actual.read("README.md"), b"viewer readme")
                    self.assertEqual(
                        actual.read("MODEL_LICENSE_README.md"), b"model notice"
                    )
            else:
                with tarfile.open(archive, "r:gz") as actual:
                    self.assertEqual(
                        set(actual.getnames()), release._expected_members(target)
                    )
                    self.assertEqual(actual.extractfile("README.md").read(), b"viewer readme")
                    self.assertEqual(
                        actual.extractfile("MODEL_LICENSE_README.md").read(), b"model notice"
                    )
        self.assertEqual(run.call_count, 2 * len(release.TARGETS))
        for index in range(0, run.call_count, 2):
            self.assertEqual(run.call_args_list[index].args[0][-1], "--version")
            self.assertTrue(run.call_args_list[index].kwargs["capture_output"])

    @mock.patch.object(release.subprocess, "run")
    def test_package_rejects_stale_version_output(self, run):
        target = release.TARGETS[0]
        inputs = self.root / "target" / target / "dist"
        inputs.mkdir(parents=True)
        for binary in release.BINARIES:
            (inputs / binary).write_bytes(b"fake")
        run.return_value = SimpleNamespace(stdout="rocketsim-viewer 0.2.9\n")
        with self.assertRaisesRegex(release.ReleaseError, "Version smoke test returned"):
            release.package(self.root, target)

    def test_verify_all_targets_and_windows_executables(self):
        for target in release.TARGETS:
            self._make_archive(target)
        self.assertEqual(release._archive_members(self.root / "dist" / f"rocketsim_viewer-0.3.0-{release.TARGETS[1]}.zip"), release._expected_members(release.TARGETS[1]))
        sums = release.verify(self.root, "v0.3.0")
        self.assertEqual(len(sums.read_text(encoding="ascii").splitlines()), 4)

    def test_verify_rejects_corrupt_checksum(self):
        archive = self._make_archive(release.TARGETS[0])
        archive.with_name(archive.name + ".sha256").write_text("bad\n", encoding="utf-8")
        with self.assertRaisesRegex(release.ReleaseError, "Checksum is invalid"):
            release.verify(self.root, "v0.3.0")

    def test_verify_rejects_unexpected_asset(self):
        for target in release.TARGETS:
            self._make_archive(target)
        (self.root / "dist" / "stray.txt").write_text("stray", encoding="utf-8")
        with self.assertRaisesRegex(release.ReleaseError, "Unexpected release assets"):
            release.verify(self.root, "v0.3.0")

    def test_verify_rejects_symlink_checksum(self):
        archive = self._make_archive(release.TARGETS[0])
        checksum = archive.with_name(archive.name + ".sha256")
        checksum.unlink()
        try:
            checksum.symlink_to(self.root / "LICENSE")
        except (NotImplementedError, OSError) as exc:
            self.skipTest(f"Cannot create symlinks on this platform: {exc}")
        with self.assertRaisesRegex(release.ReleaseError, "regular checksum"):
            release.verify(self.root, "v0.3.0")

    def test_safe_members_reject_traversal(self):
        with self.assertRaisesRegex(release.ReleaseError, "Unsafe archive path"):
            release._safe_members(["../outside"])


if __name__ == "__main__":
    unittest.main()
