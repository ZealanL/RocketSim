import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from prepare_meshes import EXPECTED_COUNTS, prepare


class PrepareMeshesTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.source = self.root / "source"
        self.destination = self.root / "destination"
        self.make_source()

    def make_source(self):
        for mode, count in EXPECTED_COUNTS.items():
            folder = self.source / mode
            folder.mkdir(parents=True, exist_ok=True)
            for index in range(count):
                (folder / f"mesh_{index}.cmf").write_bytes(f"{mode}-{index}".encode())

    def create_symlink(self, target, link, *, target_is_directory=False):
        try:
            link.symlink_to(target, target_is_directory=target_is_directory)
        except (OSError, NotImplementedError) as error:
            self.skipTest(f"Symlink creation is not available: {error}")

    def test_copies_expected_files(self):
        prepare(self.source, self.destination)
        self.assertEqual(
            {
                p.relative_to(self.destination).as_posix()
                for p in self.destination.rglob("*.cmf")
            },
            {
                f"{mode}/mesh_{index}.cmf"
                for mode, count in EXPECTED_COUNTS.items()
                for index in range(count)
            },
        )

    def test_copy_failure_does_not_publish_or_leave_staging(self):
        with patch(
            "prepare_meshes.shutil.copyfile", side_effect=OSError("copy failed")
        ):
            with self.assertRaises(OSError):
                prepare(self.source, self.destination)
        self.assertFalse(self.destination.exists())
        self.assertEqual(list(self.root.glob(".destination.*")), [])

    def test_incomplete_input_does_not_publish(self):
        (self.source / "soccar" / "mesh_0.cmf").unlink()
        with self.assertRaises(ValueError):
            prepare(self.source, self.destination)
        self.assertFalse(self.destination.exists())

    def test_symlink_source_file_is_rejected(self):
        file = self.source / "soccar" / "mesh_0.cmf"
        file.unlink()
        self.create_symlink(self.source / "soccar" / "mesh_1.cmf", file)
        with self.assertRaises(ValueError):
            prepare(self.source, self.destination)
        self.assertFalse(self.destination.exists())

    def test_destination_symlink_is_not_followed(self):
        protected = self.root / "protected"
        protected.mkdir()
        self.create_symlink(protected, self.destination, target_is_directory=True)
        with self.assertRaises(ValueError):
            prepare(self.source, self.destination)
        self.assertEqual(list(protected.iterdir()), [])

    def test_matching_existing_directory_is_reused(self):
        prepare(self.source, self.destination)
        marker = self.destination / "soccar" / "mesh_0.cmf"
        before = marker.stat().st_mtime_ns
        prepare(self.source, self.destination)
        self.assertEqual(marker.stat().st_mtime_ns, before)

    def test_mismatching_existing_directory_is_unchanged(self):
        prepare(self.source, self.destination)
        marker = self.destination / "soccar" / "mesh_0.cmf"
        marker.write_bytes(b"user data")
        with self.assertRaises(ValueError):
            prepare(self.source, self.destination)
        self.assertEqual(marker.read_bytes(), b"user data")

    def test_existing_extra_file_is_rejected_without_deletion(self):
        self.destination.mkdir()
        marker = self.destination / "keep.txt"
        marker.write_text("user data")
        with self.assertRaises(ValueError):
            prepare(self.source, self.destination)
        self.assertEqual(marker.read_text(), "user data")


if __name__ == "__main__":
    unittest.main()
