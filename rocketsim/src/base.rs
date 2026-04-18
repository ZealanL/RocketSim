use std::{
    fs,
    io::{Error as IoError, ErrorKind, Result as IoResult},
    path::Path,
    sync::{Arc, OnceLock, RwLock},
    time::Instant,
};

use ahash::AHashMap;
use log::{error, info, warn};

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

pub(crate) static ARENA_COLLISION_SHAPES: RwLock<
    Option<AHashMap<GameMode, Vec<Arc<BvhTriangleMeshShape>>>>,
> = RwLock::new(None);
pub(crate) static ARENA_COLLISION_MESH_FILES: RwLock<
    Option<AHashMap<GameMode, Vec<CollisionMeshFile>>>,
> = RwLock::new(None);

pub fn is_initialized() -> bool {
    HAS_INITIALIZED_LOCK.get().is_some()
}

pub fn init_from_default(silent: bool) -> IoResult<()> {
    init(COLLISION_MESH_BASE_PATH, silent)
}

pub fn init<P: AsRef<Path>>(collision_meshes_folder: P, silent: bool) -> IoResult<()> {
    init_from_path(collision_meshes_folder.as_ref(), silent)
}

fn init_from_path(collision_meshes_folder: &Path, silent: bool) -> IoResult<()> {
    const GAMEMODES_WITH_UNIQUE_MESHES: [GameMode; 3] =
        [GameMode::Soccar, GameMode::Hoops, GameMode::Dropshot];

    if !collision_meshes_folder.exists() {
        return Err(IoError::new(
            ErrorKind::NotFound,
            format!("{} does not exist", collision_meshes_folder.display()),
        ));
    }

    if !collision_meshes_folder.is_dir() {
        return Err(IoError::new(
            ErrorKind::NotADirectory,
            format!("{} is not a directory", collision_meshes_folder.display()),
        ));
    }

    let mut mesh_file_map = AHashMap::new();

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

pub fn init_from_mem(
    byte_mesh_file_map: AHashMap<GameMode, Vec<Vec<u8>>>,
    silent: bool,
) -> IoResult<()> {
    if !silent {
        let _ = logging::try_init();
    }

    if HAS_INITIALIZED_LOCK.set(()).is_ok() {
        // Initialization locked successfully
    } else {
        warn!("RocketSim initialized again, ignoring...");
        return Ok(());
    }

    info!("Initializing RocketSim, originally by ZealanL and ported to Rust by VirxEC...");

    let start_time = Instant::now();

    // TODO: DropshotTiles::Init();

    let mut arena_collision_shapes = AHashMap::new();
    let mut arena_collision_mesh_files = AHashMap::new();

    for (game_mode, byte_mesh_files) in byte_mesh_file_map {
        info!("Loading arena meshes for {}...", game_mode.name());

        if byte_mesh_files.is_empty() {
            info!("\tNo meshes, skipping");
            continue;
        }

        let mut meshes = Vec::with_capacity(byte_mesh_files.len());
        let mut mesh_files = Vec::with_capacity(byte_mesh_files.len());
        let mut target_hashes = game_mode.get_hashes();

        for (i, entry) in byte_mesh_files.into_iter().enumerate() {
            let mesh_file = CollisionMeshFile::read_from_bytes(&entry)?;
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

            let tri_mesh = mesh_file.make_bullet_mesh();
            let bvt_mesh = BvhTriangleMeshShape::new(tri_mesh);
            meshes.push(Arc::new(bvt_mesh));
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
