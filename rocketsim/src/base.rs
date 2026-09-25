use std::{
    fs,
    io::{Error as IoError, ErrorKind, Result as IoResult},
    path::Path,
    sync::{Arc, Mutex, OnceLock, RwLock},
    time::Instant,
};

use glam::Vec3A;
use log::{error, info, warn};
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
/// directory, or a mesh file cannot be read/parsed.
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
/// Initialization runs at most once per process: later calls log a warning
/// and return `Ok(())` without replacing the loaded meshes.
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

        for entry in fs::read_dir(folder)?.flatten() {
            let p = entry.path();

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
/// raw contents of each `.cmf` file for that mode. Modes with no entry (or
/// an empty list) are skipped.
///
/// An empty mesh list for a mode simply leaves that mode unloaded — creating
/// an [`crate::Arena`] for it later will panic. `TheVoid` never needs meshes.
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

    let _initializing_lock = INITIALIZING_MUTEX.lock();

    if HAS_INITIALIZED_LOCK.set(()).is_ok() {
        // Initialization locked successfully
    } else {
        warn!("RocketSim initialized again, ignoring...");
        return Ok(());
    }

    info!("Initializing RocketSim, originally by ZealanL and ported to Rust by VirxEC...");

    let start_time = Instant::now();

    // TODO: DropshotTiles::Init();

    let mut arena_collision_shapes = FxHashMap::default();
    let mut arena_collision_mesh_files = FxHashMap::default();

    for (game_mode, byte_mesh_files) in byte_mesh_file_map {
        info!("Loading arena meshes for {}...", game_mode.name());

        let mut byte_mesh_files = byte_mesh_files.into_iter().peekable();

        if byte_mesh_files.peek().is_none() {
            info!("\tNo meshes, skipping");
            continue;
        }

        let mut meshes = Vec::new();
        let mut mesh_files = Vec::new();
        let mut target_hashes = game_mode.get_hashes();

        for (i, entry) in byte_mesh_files.enumerate() {
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
                error!(
                    "Collision mesh [{i}] is a duplicate ({hash:#x}), already loaded a mesh with the same hash."
                );
            }

            *hash_count += 1;

            let translation = mesh_file.component_translation();
            let tri_mesh = mesh_file.make_bullet_mesh_local();
            let bvt_mesh = BvhTriangleMeshShape::new(tri_mesh);
            meshes.push(ArenaCollisionMesh {
                shape: Arc::new(bvt_mesh),
                translation,
            });
            mesh_files.push(mesh_file);
        }

        arena_collision_shapes.insert(game_mode, meshes);
        arena_collision_mesh_files.insert(game_mode, mesh_files);
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

    Ok(())
}
