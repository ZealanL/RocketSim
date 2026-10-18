# GitHub Actions and releases

## Continuous integration

`.github/workflows/ci.yml` runs on branch pushes and pull requests. You can
also run it from the GitHub Actions page. The release workflow calls the same
checks before it builds release binaries.

The checks include:

- Rust formatting and Clippy with warnings treated as errors.
- Workspace compilation on Rust 1.95, the minimum supported Rust version.
- A core-library build without default features.
- Tests on Linux, Windows, and Apple Silicon macOS.
- All-feature tests on Linux, including the optional C++ comparison backend.
- Documentation tests and a headless viewer `--help` check.
- Release metadata, archive-tool, and mesh-fixture tests.
- Core source-package verification.

Linux tests run serially. The optional C++ comparison backend uses
process-global state and can fail when its tests run concurrently.

The jobs use the root `Cargo.lock` with `--locked`. Keep this file in Git.
GitHub Actions dependencies are pinned to commit IDs. Update the IDs when you
update the corresponding actions.

## Collision mesh fixtures

The repository does not track the arena collision meshes. Test jobs check out
[RLGym/rlgym](https://github.com/RLGym/rlgym) at commit
`34c0d2a388d5ae50466cbeb02b4c4ff367e7a054`. They copy its
`rlgym/rocket_league/sim/collision_meshes` directory with
`tools/ci/prepare_meshes.py`.

The helper requires 16 Soccar, 12 Hoops, and 2 Dropshot files. The engine also
validates the component hashes during initialization. The helper does not
replace an existing directory with different contents. It does not follow
source or destination symlinks.

These fixtures are test inputs. The release archives do not contain them.
The simulation still needs its own collision meshes. The detached viewer
receives scene geometry from the simulation.

## Prebuilt viewer targets

| Target | Runner | Archive |
| --- | --- | --- |
| `x86_64-unknown-linux-gnu` | Ubuntu 22.04 | `.tgz` |
| `x86_64-pc-windows-msvc` | Windows Server 2022 | `.zip` |
| `x86_64-apple-darwin` | Intel macOS 15 | `.tgz` |
| `aarch64-apple-darwin` | Apple Silicon macOS 14 | `.tgz` |

Linux binaries require glibc 2.35 or later. They also need `libudev`,
`libxkbcommon`, OpenGL/EGL, and an X11 or Wayland desktop. Windows binaries
use the static MSVC C runtime. macOS binaries target macOS 11 or later.
This workflow does not sign or notarize the binaries.

Each archive contains:

- `rocketsim-viewer` and `mesh_stats`, with `.exe` suffixes on Windows.
- The viewer `README.md`.
- The code `LICENSE` and `MODEL_LICENSE_README.md`.

The `dist` Cargo profile removes debug data and strips symbols. Each archive
has a SHA-256 checksum file. The draft release also has a `SHA256SUMS` file.

## Create a release

1. Update `workspace.package.version` in `Cargo.toml`.
2. Update the internal dependency versions in `rocketsim_vis/Cargo.toml` and
   `rocketsim_viewer/Cargo.toml`.
3. Update the root `Cargo.lock`. Commit the version and lockfile changes.
4. Wait for CI to pass.
5. Push a version tag that exactly matches the workspace version. For example:

   ```sh
   git tag -a v0.3.0 -m "RocketSim 0.3.0"
   git push origin v0.3.0
   ```

`.github/workflows/release.yml` accepts `v0.*` tags. It checks the version,
runs CI, and builds all four targets with Rust 1.95. It tests the viewer's
`--version` and `--help` output without opening a window. It then checks the
archive contents and checksums before it creates a draft GitHub Release.

The workflow can update an existing draft. It refuses to replace assets on
a published release. Do not move a published version tag to another commit.

Only the draft-release job has `contents: write` permission. It uses the
repository's automatic `GITHUB_TOKEN`. No personal access token or crates.io
secret is needed by these workflows. Repository or organization policies must
permit the workflow to create releases.

## Publish the release

The workflow does not publish crates or publish the draft automatically.

1. Review the draft assets and the code and model license notices.
2. Publish the library crates in dependency order:

   ```sh
   cargo publish --locked -p rocketsim
   cargo publish --locked -p rocketsim_vis
   ```

3. Publish the draft GitHub Release. Mark prerelease versions as prereleases.
4. Publish the viewer crate after its dependencies are available on crates.io:

   ```sh
   cargo publish --locked -p rocketsim_viewer
   ```

The public GitHub assets must be available before users install the viewer
with `cargo binstall`. Its metadata uses this URL pattern:

```text
https://github.com/ZealanL/RocketSim/releases/download/vVERSION/rocketsim_viewer-VERSION-TARGET.tgz
```

Windows uses the same pattern with `.zip`. Binstall reads the exact version
from the published crate metadata. Unsupported targets can compile from source.
Third-party QuickInstall downloads are disabled for this package.

Verify the published installation with:

```sh
cargo binstall rocketsim_viewer --version 0.3.0
rocketsim-viewer --version
```

## Local checks

Use Python 3.12 or later for the helper scripts.

```sh
cargo fmt --all -- --check
cargo clippy --locked --workspace --all-targets --all-features -- -D warnings
cargo test --locked --workspace --all-targets --all-features -- --test-threads=1
cargo test --locked --workspace --all-features --doc
python3 -m unittest discover -s tools/ci -v
python3 tools/ci/release.py check
```

The Rust tests need the collision meshes at `collision_meshes/`.

To build and package a native Linux archive:

```sh
cargo +1.95 build --locked --profile dist -p rocketsim_viewer --bins --target x86_64-unknown-linux-gnu
python3 tools/ci/release.py package --target x86_64-unknown-linux-gnu
```

The helper writes the archive and checksum into `dist/`. It does not upload
files or create a GitHub Release. The `verify --tag vVERSION` command requires
all four target archives and their checksum files in `dist/`.
