use std::{
    io,
    path::{Path, PathBuf},
    process::{Command, Stdio},
};

use rocketsim_vis::{
    DEFAULT_VIEWER_ADDR, ViewerHandle, launcher::try_ensure_viewer_running_with_path,
};
use serde::Deserialize;

/// Build the workspace viewer without adding GUI dependencies to the transport crate.
pub fn prepare_viewer() -> io::Result<Option<ViewerHandle>> {
    if std::env::var_os("ROCKETSIM_VIEWER").is_some() {
        return Ok(None);
    }
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR")).join("../rocketsim_viewer/Cargo.toml");
    if !manifest.is_file() {
        return Ok(None);
    }
    let exe = std::env::current_exe()?;
    let profile = build_profile(&exe)?;
    eprintln!("Building the workspace viewer ({profile})...");
    let output = Command::new(env!("CARGO"))
        .current_dir(env!("CARGO_MANIFEST_DIR"))
        .arg("build")
        .arg("--manifest-path")
        .arg(manifest)
        .args(["--bin", "rocketsim-viewer", "--profile", profile])
        .arg("--message-format=json-render-diagnostics")
        .stderr(Stdio::inherit())
        .output()?;
    if !output.status.success() {
        return Err(io::Error::other(format!(
            "workspace viewer build failed with {}",
            output.status
        )));
    }
    let path = viewer_artifact(&output.stdout)?;
    try_ensure_viewer_running_with_path(DEFAULT_VIEWER_ADDR, &path).map(Some)
}

fn build_profile(exe: &Path) -> io::Result<&str> {
    let profile = exe
        .parent()
        .filter(|dir| dir.file_name().is_some_and(|name| name == "examples"))
        .and_then(Path::parent)
        .and_then(Path::file_name)
        .and_then(|name| name.to_str())
        .ok_or_else(|| {
            io::Error::other(format!(
                "cannot find the Cargo profile for {}; set ROCKETSIM_VIEWER to a viewer executable",
                exe.display()
            ))
        })?;
    Ok(if profile == "debug" { "dev" } else { profile })
}

#[derive(Deserialize)]
struct Artifact {
    reason: String,
    target: Option<ArtifactTarget>,
    executable: Option<PathBuf>,
}

#[derive(Deserialize)]
struct ArtifactTarget {
    name: String,
    kind: Vec<String>,
}

fn viewer_artifact(output: &[u8]) -> io::Result<PathBuf> {
    output
        .split(|&byte| byte == b'\n')
        .filter_map(|line| serde_json::from_slice::<Artifact>(line).ok())
        .find_map(|artifact| {
            (artifact.reason == "compiler-artifact"
                && artifact.target.is_some_and(|target| {
                    target.name == "rocketsim-viewer"
                        && target.kind.iter().any(|kind| kind == "bin")
                }))
            .then_some(artifact.executable)
            .flatten()
        })
        .ok_or_else(|| io::Error::other("Cargo did not report a rocketsim-viewer executable"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profiles_support_debug_release_custom_and_target_triples() {
        for (exe, expected) in [
            ("target/debug/examples/drive", "dev"),
            ("target/release/examples/drive", "release"),
            ("custom-target/fast/examples/drive", "fast"),
            (
                "target/x86_64-unknown-linux-gnu/release/examples/drive",
                "release",
            ),
        ] {
            assert_eq!(build_profile(Path::new(exe)).unwrap(), expected);
        }
        assert!(build_profile(Path::new("copied/drive")).is_err());
    }

    #[test]
    fn select_the_reported_viewer_binary_not_another_artifact() {
        let output = br#"{"reason":"compiler-artifact","target":{"name":"build-script-build","kind":["custom-build"]},"executable":"other/build-script"}
{"reason":"compiler-artifact","target":{"name":"rocketsim_viewer","kind":["lib"]},"executable":null}
{"reason":"compiler-artifact","target":{"name":"rocketsim-viewer","kind":["bin"]},"executable":"custom target/release/rocketsim-viewer"}
{"reason":"build-finished","success":true}
"#;
        assert_eq!(
            viewer_artifact(output).unwrap(),
            Path::new("custom target/release/rocketsim-viewer")
        );
        assert!(viewer_artifact(br#"{"reason":"build-finished","success":true}"#).is_err());
    }
}
