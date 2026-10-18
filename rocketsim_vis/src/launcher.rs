//! Viewer discovery and ownership. Arena connections never share a viewer address.

use std::{
    collections::HashMap,
    io,
    net::{SocketAddr, TcpStream},
    path::{Path, PathBuf},
    process::{Child, Command},
    sync::{Mutex, OnceLock},
    thread,
    time::{Duration, Instant},
};

#[derive(Default)]
struct Entry {
    child: Option<Child>,
    references: usize,
    attached: bool,
    keepalive: bool,
}

fn registry() -> &'static Mutex<HashMap<SocketAddr, Entry>> {
    static REGISTRY: OnceLock<Mutex<HashMap<SocketAddr, Entry>>> = OnceLock::new();
    REGISTRY.get_or_init(|| Mutex::new(HashMap::new()))
}

pub(crate) struct ViewerLease {
    addr: SocketAddr,
    attached: bool,
}

impl Drop for ViewerLease {
    fn drop(&mut self) {
        let child = {
            let Ok(mut entries) = registry().lock() else {
                return;
            };
            let Some(entry) = entries.get_mut(&self.addr) else {
                return;
            };
            entry.references = entry.references.saturating_sub(1);
            if self.attached {
                entry.attached = false;
            }
            if entry.references == 0 && !entry.keepalive {
                entries.remove(&self.addr).and_then(|entry| entry.child)
            } else {
                None
            }
        };
        if let Some(mut child) = child {
            let _ = child.kill();
            // Reap the process even if it exited before kill().
            let _ = child.wait();
        }
    }
}

fn parse(addr: &str) -> io::Result<SocketAddr> {
    addr.parse().map_err(|error| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("viewer address must be numeric (IP:port): {error}"),
        )
    })
}

fn probe(addr: SocketAddr) -> bool {
    TcpStream::connect_timeout(&addr, Duration::from_millis(100)).is_ok()
}

fn reserve(addr: SocketAddr, attached: bool) -> io::Result<ViewerLease> {
    let mut entries = registry()
        .lock()
        .map_err(|_| io::Error::other("viewer process registry is unavailable"))?;
    let entry = entries.entry(addr).or_default();
    if attached && entry.attached {
        return Err(io::Error::new(
            io::ErrorKind::AddrInUse,
            format!("another arena uses viewer {addr}; use a separate viewer and port"),
        ));
    }
    entry.references += 1;
    entry.attached |= attached;
    Ok(ViewerLease { addr, attached })
}

/// Reserve a viewer for one arena. A connection-only attachment does not launch a process.
pub(crate) fn attach(addr: SocketAddr, launch: bool) -> io::Result<ViewerLease> {
    let lease = reserve(addr, true)?;
    if launch {
        start_and_wait(addr, None)?;
    }
    Ok(lease)
}

/// Ensure a viewer is available. Keep this handle while the viewer is needed.
///
/// The last handle or arena attachment closes a viewer started by this crate.
/// A viewer started by the user is never closed by this crate.
pub fn try_ensure_viewer_running(addr: &str) -> io::Result<ViewerHandle> {
    let addr = parse(addr)?;
    let lease = reserve(addr, false)?;
    start_and_wait(addr, None)?;
    Ok(ViewerHandle { _lease: lease })
}

/// Ensure a viewer is available. Use `path` if a new process is needed.
///
/// This does not search `PATH` or use `ROCKETSIM_VIEWER`. A viewer that already
/// listens at `addr` is reused. Keep the returned handle while the viewer is needed.
pub fn try_ensure_viewer_running_with_path(addr: &str, path: &Path) -> io::Result<ViewerHandle> {
    let addr = parse(addr)?;
    let lease = reserve(addr, false)?;
    start_and_wait(addr, Some(path))?;
    Ok(ViewerHandle { _lease: lease })
}

/// A process lease. Dropping the handle releases only this lease.
pub struct ViewerHandle {
    _lease: ViewerLease,
}

/// Start a standalone viewer on a best-effort basis.
///
/// This compatibility function leaves the viewer running. Use
/// [`try_ensure_viewer_running`] for scoped ownership and error handling.
pub fn ensure_viewer_running(addr: &str) {
    match try_ensure_viewer_running(addr) {
        Ok(handle) => {
            if let Ok(mut entries) = registry().lock()
                && let Some(entry) = entries.get_mut(&handle._lease.addr)
            {
                entry.keepalive = true;
            }
        }
        Err(error) => eprintln!("rocketsim_vis: {error}"),
    }
}

fn start_and_wait(addr: SocketAddr, viewer_path: Option<&Path>) -> io::Result<()> {
    {
        let mut entries = registry()
            .lock()
            .map_err(|_| io::Error::other("viewer process registry is unavailable"))?;
        let entry = entries
            .get_mut(&addr)
            .ok_or_else(|| io::Error::other("viewer lease is missing"))?;
        if let Some(child) = entry.child.as_mut()
            && child.try_wait()?.is_some()
        {
            entry.child = None;
        }
        if probe(addr) {
            return Ok(());
        }
        if !addr.ip().is_loopback() || addr.port() == 0 {
            return Err(io::Error::new(
                io::ErrorKind::AddrNotAvailable,
                format!("cannot launch a viewer at {addr}; start remote viewers explicitly"),
            ));
        }
        if entry.child.is_none() {
            let path = viewer_path.map(Path::to_path_buf).or_else(find_viewer).ok_or_else(|| {
                io::Error::new(
                    io::ErrorKind::NotFound,
                    "rocketsim-viewer was not found; install it on PATH or run `cargo build -p rocketsim_viewer --bin rocketsim-viewer`",
                )
            })?;
            entry.child = Some(
                Command::new(&path)
                    .arg(addr.port().to_string())
                    .arg("--bind")
                    .arg(addr.ip().to_string())
                    .spawn()
                    .map_err(|error| {
                        io::Error::new(
                            error.kind(),
                            format!("cannot start {}: {error}", path.display()),
                        )
                    })?,
            );
        }
    }
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        {
            let mut entries = registry()
                .lock()
                .map_err(|_| io::Error::other("viewer process registry is unavailable"))?;
            if let Some(child) = entries
                .get_mut(&addr)
                .and_then(|entry| entry.child.as_mut())
                && let Some(status) = child.try_wait()?
            {
                return Err(io::Error::other(format!(
                    "rocketsim-viewer exited with {status} before listening on {addr}"
                )));
            }
        }
        if probe(addr) {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                format!("rocketsim-viewer did not listen on {addr} within five seconds"),
            ));
        }
        thread::sleep(Duration::from_millis(25));
    }
}

fn binary_name() -> &'static str {
    if cfg!(windows) {
        "rocketsim-viewer.exe"
    } else {
        "rocketsim-viewer"
    }
}

fn sibling_candidates(exe: &Path) -> Vec<PathBuf> {
    let Some(dir) = exe.parent() else {
        return Vec::new();
    };
    let mut paths = vec![dir.join(binary_name())];
    if matches!(
        dir.file_name().and_then(|name| name.to_str()),
        Some("examples" | "deps")
    ) && let Some(parent) = dir.parent()
    {
        paths.push(parent.join(binary_name()));
    }
    paths
}

fn executable(path: &Path) -> bool {
    let Ok(metadata) = path.metadata() else {
        return false;
    };
    if !metadata.is_file() {
        return false;
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        metadata.permissions().mode() & 0o111 != 0
    }
    #[cfg(not(unix))]
    {
        true
    }
}

fn find_viewer() -> Option<PathBuf> {
    if let Some(path) = std::env::var_os("ROCKETSIM_VIEWER") {
        return executable(Path::new(&path)).then(|| PathBuf::from(path));
    }
    viewer_candidates(
        std::env::current_exe().ok().as_deref(),
        std::env::var_os("PATH").as_deref(),
    )
    .into_iter()
    .find(|path| executable(path))
}

fn viewer_candidates(exe: Option<&Path>, search_path: Option<&std::ffi::OsStr>) -> Vec<PathBuf> {
    let mut candidates = exe.map(sibling_candidates).unwrap_or_default();
    if let Some(paths) = search_path {
        candidates.extend(
            std::env::split_paths(paths)
                .filter(|path| !path.as_os_str().is_empty())
                .map(|path| path.join(binary_name())),
        );
    }
    candidates
}

#[cfg(test)]
mod tests {
    use std::net::TcpListener;

    use super::*;

    #[test]
    fn discovery_supports_regular_examples_and_test_executables() {
        let name = binary_name();
        assert_eq!(
            sibling_candidates(Path::new("target/debug/sim")),
            vec![Path::new("target/debug").join(name)]
        );
        for dir in ["examples", "deps"] {
            let exe = Path::new("target/debug").join(dir).join("sim");
            assert!(sibling_candidates(&exe).contains(&Path::new("target/debug").join(name)));
        }
    }

    #[test]
    fn local_viewers_precede_path_candidates() {
        let search_path = std::env::join_paths(["installed/bin", "other/bin"]).unwrap();
        let local = Path::new("target/release").join(binary_name());
        let installed = Path::new("installed/bin").join(binary_name());
        let candidates = viewer_candidates(
            Some(Path::new("target/release/examples/drive")),
            Some(&search_path),
        );
        let local_index = candidates.iter().position(|path| path == &local).unwrap();
        let installed_index = candidates
            .iter()
            .position(|path| path == &installed)
            .unwrap();
        assert!(local_index < installed_index);
        assert_eq!(viewer_candidates(None, Some(&search_path))[0], installed);
    }

    #[test]
    fn explicit_viewer_path_is_used_and_failed_launch_releases_lease() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        drop(listener);
        let path = Path::new("missing-viewer-directory/rocketsim-viewer");
        let error = try_ensure_viewer_running_with_path(&addr.to_string(), path)
            .err()
            .unwrap();
        assert_eq!(error.kind(), io::ErrorKind::NotFound);
        assert!(error.to_string().contains(&path.display().to_string()));
        assert!(!registry().lock().unwrap().contains_key(&addr));
    }

    #[test]
    fn attachments_are_exclusive_and_handle_leases_are_independent() {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let addr = listener.local_addr().unwrap();
        let attachment = attach(addr, false).unwrap();
        assert_eq!(
            attach(addr, false).err().unwrap().kind(),
            io::ErrorKind::AddrInUse
        );
        let handle = reserve(addr, false).unwrap();
        drop(attachment);
        let replacement = attach(addr, false).unwrap();
        drop(handle);
        assert!(registry().lock().unwrap().get(&addr).unwrap().attached);
        drop(replacement);
        assert!(!registry().lock().unwrap().contains_key(&addr));
        assert!(
            TcpStream::connect(addr).is_ok(),
            "a user-owned listener was closed"
        );
    }

    #[test]
    fn dropping_an_unrelated_lease_keeps_the_other_attachment() {
        let first_listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let second_listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let first_addr = first_listener.local_addr().unwrap();
        let second_addr = second_listener.local_addr().unwrap();
        let first = attach(first_addr, false).unwrap();
        let second = attach(second_addr, false).unwrap();
        drop(second);
        assert!(
            registry()
                .lock()
                .unwrap()
                .get(&first_addr)
                .unwrap()
                .attached
        );
        drop(first);
    }
}
