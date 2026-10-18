use std::{
    fs,
    io::{Error as IoError, ErrorKind, Result as IoResult},
    path::Path,
    sync::{Arc, Mutex, OnceLock, RwLock},
    time::Instant,
};

use glam::Vec3A;
use log::{info, warn};
use rustc_hash::FxHashMap;

use crate::{
    bullet::collision::shapes::bvh_triangle_mesh_shape::BvhTriangleMeshShape,
    logging,
    sim::{
        GameMode,
        collision_mesh_file::{
            COLLISION_MESH_BASE_PATH, COLLISION_MESH_FILE_EXTENSION, CollisionMeshFile,
        },
    },
};

static HAS_INITIALIZED_LOCK: OnceLock<()> = OnceLock::new();
static INITIALIZING_MUTEX: Mutex<()> = Mutex::new(());

/// Arena mesh with its recovered component translation.
/// Goal components use local vertices plus (0, +/-102.4, 0) BT.
/// Other components use identity.
#[derive(Clone)]
pub(crate) struct ArenaCollisionMesh {
    pub shape: Arc<BvhTriangleMeshShape>,
    pub translation: Vec3A,
}

pub(crate) static ARENA_COLLISION_SHAPES: RwLock<
    Option<FxHashMap<GameMode, Vec<ArenaCollisionMesh>>>,
> = RwLock::new(None);
pub(crate) static ARENA_COLLISION_MESH_FILES: RwLock<
    Option<FxHashMap<GameMode, Vec<CollisionMeshFile>>>,
> = RwLock::new(None);

/// Returns `true` after a successful [`init`]/[`init_from_default`]/[`init_from_mem`] call.
///
/// Useful to guard one-time setup in tests and binaries that may run
/// initialization from multiple entry points.
pub fn is_initialized() -> bool {
    HAS_INITIALIZED_LOCK.get().is_some()
}

/// Returns a copy of the loaded `.cmf` files for a game mode.
///
/// `Heatseeker`, `Snowday`, and `Soccar` all share the Soccar meshes.
/// `TheVoid` has no arena hull, so this always returns an empty [`Vec`].
///
/// # Panics
///
/// Panics if RocketSim was not initialized with [`init`] first.
pub fn get_arena_collision_mesh_files(game_mode: GameMode) -> Vec<CollisionMeshFile> {
    let collision_mesh_files = ARENA_COLLISION_MESH_FILES.read().unwrap();
    let meshes_mode = match game_mode {
        GameMode::Hoops => game_mode,
        GameMode::Dropshot => game_mode,
        GameMode::TheVoid => return Vec::new(),
        _ => GameMode::Soccar,
    };

    collision_mesh_files.as_ref().unwrap()[&meshes_mode].clone()
}

/// Initializes RocketSim from the default `./collision_meshes/` folder.
///
/// Equivalent to `init("./collision_meshes/", silent)`. This is the normal
/// entry point for games and examples.
///
/// # Arguments
///
/// * `silent` - when `false`, installs the default `stderr` logger; pass
///   `true` to keep initialization quiet (e.g. in tests).
///
/// # Errors
///
/// Returns an [`std::io::Error`] if the folder is missing, is not a
/// directory, or a mesh file cannot be read/parsed. Returns `InvalidData` if
/// a supplied mode has duplicate or missing collision mesh components.
///
/// # Example
///
/// ```no_run
/// rocketsim::init_from_default(true).unwrap();
/// assert!(rocketsim::is_initialized());
/// ```
pub fn init_from_default(silent: bool) -> IoResult<()> {
    init(COLLISION_MESH_BASE_PATH, silent)
}

/// Initializes RocketSim from a custom collision-mesh folder.
///
/// The folder is expected to contain one subfolder per game mode
/// (`soccar/`, `hoops/`, `dropshot/`, see [`GameMode::name`]) with `.cmf`
/// files inside. Unknown hashes are skipped with a warning; see
/// [`CollisionMeshFile::read_from_bytes`].
///
/// Successful initialization runs at most once per process. Later calls log a
/// warning and return `Ok(())` without replacing the loaded meshes. Failed
/// initialization does not publish meshes and can be retried.
///
/// # Arguments
///
/// * `collision_meshes_folder` - e.g. `"./collision_meshes"` or a path
///   baked in with `env!("CARGO_MANIFEST_DIR")`.
/// * `silent` - see [`init_from_default`].
///
/// # Example
///
/// ```no_run
/// rocketsim::init("./collision_meshes", true).unwrap();
/// ```
pub fn init<P: AsRef<Path>>(collision_meshes_folder: P, silent: bool) -> IoResult<()> {
    init_from_path(collision_meshes_folder.as_ref(), silent)
}

fn init_from_path(collision_meshes_folder: &Path, silent: bool) -> IoResult<()> {
    const GAMEMODES_WITH_UNIQUE_MESHES: [GameMode; 3] =
        [GameMode::Soccar, GameMode::Hoops, GameMode::Dropshot];

    if !collision_meshes_folder.exists() {
        let working_dir = std::env::current_dir().expect("Could not get working directory");

        return Err(IoError::new(
            ErrorKind::NotFound,
            format!(
                "{} does not exist (from: {})",
                collision_meshes_folder.display(),
                working_dir.display()
            ),
        ));
    }

    if !collision_meshes_folder.is_dir() {
        return Err(IoError::new(
            ErrorKind::NotADirectory,
            format!("{} is not a directory", collision_meshes_folder.display()),
        ));
    }

    let mut mesh_file_map = FxHashMap::default();

    for game_mode in GAMEMODES_WITH_UNIQUE_MESHES {
        let folder = collision_meshes_folder.join(game_mode.name());
        if !folder.exists() {
            continue;
        }

        let mut files = Vec::new();

        for entry in fs::read_dir(folder)? {
            let p = entry?.path();

            if !p.is_file()
                || p.extension()
                    .is_none_or(|ext| ext != COLLISION_MESH_FILE_EXTENSION)
            {
                continue;
            }

            files.push(fs::read(p)?);
        }

        mesh_file_map.insert(game_mode, files);
    }

    init_from_mem(mesh_file_map, silent)
}

/// Initializes RocketSim from already-loaded `.cmf` byte buffers.
///
/// Useful for embedding meshes (e.g. with `include_bytes!`) or for WASM
/// targets without filesystem access. Keys are game modes; values are the
/// raw contents of each `.cmf` file for that mode. Each supplied arena mode
/// must contain all its known components exactly once. Unknown components
/// are skipped. `Heatseeker` and `Snowday` use the `Soccar` mesh set.
/// `TheVoid` does not load meshes.
///
/// Modes with no entry remain unloaded. Creating an [`crate::Arena`] for an
/// unloaded mode will panic. Initialization publishes no meshes on failure,
/// so callers can correct the input and retry.
///
/// # Errors
///
/// Returns an I/O error on truncated data. Returns `InvalidData` for duplicate
/// or missing components, including an empty list for a supplied arena mode.
/// Returns `InvalidInput` if the same mesh mode is supplied more than once.
///
/// See [`init`] for the once-only semantics and the `silent` flag.
pub fn init_from_mem<I, V, B>(byte_mesh_file_map: I, silent: bool) -> IoResult<()>
where
    I: IntoIterator<Item = (GameMode, V)>,
    V: IntoIterator<Item = B>,
    B: AsRef<[u8]>,
{
    if !silent {
        let _ = logging::try_init();
    }

    // A parser or caller-supplied iterator can panic before publication. Its
    // local data is discarded, so a poisoned initialization lock is retryable.
    let _initializing_lock = INITIALIZING_MUTEX
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());

    if is_initialized() {
        warn!("RocketSim initialized again, ignoring...");
        return Ok(());
    }

    info!("Initializing RocketSim, originally by ZealanL and ported to Rust by VirxEC...");

    let start_time = Instant::now();

    // TODO: DropshotTiles::Init();

    let mut arena_collision_mesh_files = FxHashMap::default();

    for (game_mode, byte_mesh_files) in byte_mesh_file_map {
        let game_mode = match game_mode {
            GameMode::Heatseeker | GameMode::Snowday => GameMode::Soccar,
            GameMode::TheVoid => continue,
            mode => mode,
        };
        if arena_collision_mesh_files.contains_key(&game_mode) {
            return Err(IoError::new(
                ErrorKind::InvalidInput,
                format!("{} mesh set was supplied more than once", game_mode.name()),
            ));
        }
        info!("Loading arena meshes for {}...", game_mode.name());

        let mut mesh_files = Vec::new();
        let mut target_hashes = game_mode.get_hashes();

        for (i, entry) in byte_mesh_files.into_iter().enumerate() {
            let mesh_file = CollisionMeshFile::read_from_bytes(entry.as_ref())?;
            let hash = mesh_file.get_hash();
            let Some(hash_count) = target_hashes.get_mut(&hash) else {
                warn!(
                    "Collision mesh [{i}] does not match any known {} collision mesh ({hash:#x}), \
                    make sure they were dumped form a normal {} arena.",
                    game_mode.name(),
                    game_mode.name()
                );
                continue;
            };

            if *hash_count > 0 {
                return Err(IoError::new(
                    ErrorKind::InvalidData,
                    format!(
                        "duplicate {} collision mesh component at index {i} ({hash:#010x})",
                        game_mode.name()
                    ),
                ));
            }

            *hash_count += 1;
            mesh_files.push(mesh_file);
        }

        let mut missing: Vec<_> = target_hashes
            .into_iter()
            .filter_map(|(hash, count)| (count == 0).then_some(hash))
            .collect();
        if !missing.is_empty() {
            missing.sort_unstable();
            let hashes = missing
                .iter()
                .map(|hash| format!("{hash:#010x}"))
                .collect::<Vec<_>>()
                .join(", ");
            return Err(IoError::new(
                ErrorKind::InvalidData,
                format!(
                    "incomplete {} collision mesh set: missing {} component(s): {hashes}",
                    game_mode.name(),
                    missing.len()
                ),
            ));
        }

        arena_collision_mesh_files.insert(game_mode, mesh_files);
    }

    let mut arena_collision_shapes = FxHashMap::default();
    for (&game_mode, mesh_files) in &arena_collision_mesh_files {
        let meshes = mesh_files
            .iter()
            .map(|mesh_file| ArenaCollisionMesh {
                shape: Arc::new(BvhTriangleMeshShape::new(
                    mesh_file.make_bullet_mesh_local(),
                )),
                translation: mesh_file.component_translation(),
            })
            .collect();
        arena_collision_shapes.insert(game_mode, meshes);
    }

    {
        info!("Finished loading arena collision meshes:");
        info!(
            "\tSoccar: {}",
            arena_collision_shapes
                .get(&GameMode::Soccar)
                .map_or(0, Vec::len)
        );
        info!(
            "\tHoops: {}",
            arena_collision_shapes
                .get(&GameMode::Hoops)
                .map_or(0, Vec::len)
        );
        info!(
            "\tDropshot: {}",
            arena_collision_shapes
                .get(&GameMode::Dropshot)
                .map_or(0, Vec::len)
        );

        let elapsed_time = Instant::now().duration_since(start_time);
        info!(
            "Finished initializing RocketSim in {:.3}s!",
            elapsed_time.as_secs_f32()
        );
    }

    {
        let mut arena_collision_shapes_lock = ARENA_COLLISION_SHAPES.write().unwrap();
        let mut arena_collision_mesh_files_lock = ARENA_COLLISION_MESH_FILES.write().unwrap();
        assert!(
            arena_collision_shapes_lock.is_none(),
            "RocketSim initialization called twice"
        );
        *arena_collision_shapes_lock = Some(arena_collision_shapes);
        *arena_collision_mesh_files_lock = Some(arena_collision_mesh_files);
    }
    HAS_INITIALIZED_LOCK
        .set(())
        .expect("RocketSim readiness was already published");

    Ok(())
}
