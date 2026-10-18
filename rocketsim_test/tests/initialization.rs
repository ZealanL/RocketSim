use std::{
    collections::HashMap, fs, io::ErrorKind, path::Path, sync::mpsc, thread, time::Duration,
};

use rocketsim::{Arena, GameMode};

type MeshMap = Vec<(GameMode, Vec<Vec<u8>>)>;

fn fixtures() -> MeshMap {
    [GameMode::Soccar, GameMode::Hoops, GameMode::Dropshot]
        .into_iter()
        .map(|mode| {
            let folder = Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../collision_meshes")
                .join(mode.name());
            let mut files: Vec<_> = fs::read_dir(folder)
                .unwrap()
                .map(|entry| entry.unwrap().path())
                .filter(|path| path.extension().is_some_and(|ext| ext == "cmf"))
                .map(|path| fs::read(path).unwrap())
                .collect();
            files.sort_by_key(|bytes| bytes.len());
            (mode, files)
        })
        .collect()
}

fn unknown_component() -> Vec<u8> {
    let mut bytes = Vec::new();
    for value in [1u32, 3, 0, 1, 2] {
        bytes.extend_from_slice(&value.to_le_bytes());
    }
    for value in [0.0f32, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0] {
        bytes.extend_from_slice(&value.to_le_bytes());
    }
    bytes
}

fn error(map: MeshMap, expected: ErrorKind) {
    let result = rocketsim::init_from_mem(map, true);
    assert_eq!(result.unwrap_err().kind(), expected);
    assert!(!rocketsim::is_initialized());
}

#[test]
fn initialization_is_atomic_complete_retryable_and_once_only() {
    let mut valid = fixtures();

    error(
        vec![(GameMode::Soccar, vec![vec![0; 3]])],
        ErrorKind::UnexpectedEof,
    );
    for mode in [
        GameMode::Soccar,
        GameMode::Hoops,
        GameMode::Dropshot,
        GameMode::Heatseeker,
        GameMode::Snowday,
    ] {
        error(vec![(mode, vec![])], ErrorKind::InvalidData);
    }
    error(
        vec![(GameMode::Soccar, vec![unknown_component()])],
        ErrorKind::InvalidData,
    );

    let mut duplicate = valid.clone();
    let files = &mut duplicate
        .iter_mut()
        .find(|(mode, _)| *mode == GameMode::Soccar)
        .unwrap()
        .1;
    files.push(files[0].clone());
    error(duplicate, ErrorKind::InvalidData);

    for mode in [GameMode::Soccar, GameMode::Hoops, GameMode::Dropshot] {
        let files = &valid
            .iter()
            .find(|(candidate, _)| *candidate == mode)
            .unwrap()
            .1;
        for missing in 0..files.len() {
            let mut components = files.clone();
            components.remove(missing);
            error(vec![(mode, components)], ErrorKind::InvalidData);
        }
    }

    let mut later_mode_failure = valid.clone();
    later_mode_failure
        .iter_mut()
        .find(|(mode, _)| *mode == GameMode::Dropshot)
        .unwrap()
        .1
        .pop();
    error(later_mode_failure, ErrorKind::InvalidData);
    assert!(
        std::panic::catch_unwind(|| rocketsim::get_arena_collision_mesh_files(GameMode::Soccar))
            .is_err()
    );

    let panic_map = vec![(GameMode::Soccar, vec![vec![0; 8]])];
    assert!(std::panic::catch_unwind(|| rocketsim::init_from_mem(panic_map, true)).is_err());
    assert!(!rocketsim::is_initialized());
    error(
        vec![(GameMode::Soccar, vec![vec![0; 3]])],
        ErrorKind::UnexpectedEof,
    );

    let mut alias_duplicate = vec![valid[0].clone(), valid[0].clone()];
    alias_duplicate[1].0 = GameMode::Heatseeker;
    error(alias_duplicate, ErrorKind::InvalidInput);

    valid[0].0 = GameMode::Heatseeker;
    valid[0].1.push(unknown_component());
    let (entered_tx, entered_rx) = mpsc::channel();
    let (release_tx, release_rx) = mpsc::channel();
    let (blocker_done_tx, blocker_done_rx) = mpsc::channel();
    let blocker = thread::spawn(move || {
        let mut entered_tx = Some(entered_tx);
        let input = valid.into_iter().inspect(move |_| {
            if let Some(entered_tx) = entered_tx.take() {
                entered_tx.send(()).unwrap();
                release_rx.recv_timeout(Duration::from_secs(5)).unwrap();
            }
        });
        blocker_done_tx
            .send(rocketsim::init_from_mem(input, true))
            .unwrap();
    });
    entered_rx.recv_timeout(Duration::from_secs(5)).unwrap();
    assert!(!rocketsim::is_initialized());
    let (started_tx, started_rx) = mpsc::channel();
    let (competing_done_tx, competing_done_rx) = mpsc::channel();
    let competing = thread::spawn(move || {
        started_tx.send(()).unwrap();
        competing_done_tx
            .send(rocketsim::init_from_mem(
                [(GameMode::Soccar, [vec![0; 3]])],
                true,
            ))
            .unwrap();
    });
    started_rx.recv_timeout(Duration::from_secs(5)).unwrap();
    assert!(matches!(
        competing_done_rx.recv_timeout(Duration::from_millis(50)),
        Err(mpsc::RecvTimeoutError::Timeout)
    ));
    release_tx.send(()).unwrap();
    blocker_done_rx
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap();
    competing_done_rx
        .recv_timeout(Duration::from_secs(5))
        .unwrap()
        .unwrap();
    blocker.join().unwrap();
    competing.join().unwrap();
    assert!(rocketsim::is_initialized());
    rocketsim::init_from_mem(
        std::iter::from_fn(|| -> Option<(GameMode, Vec<Vec<u8>>)> {
            panic!("successful initialization must not consume later inputs");
        }),
        true,
    )
    .unwrap();

    let counts: HashMap<_, _> = [GameMode::Soccar, GameMode::Hoops, GameMode::Dropshot]
        .into_iter()
        .map(|mode| (mode, rocketsim::get_arena_collision_mesh_files(mode).len()))
        .collect();
    assert_eq!(counts[&GameMode::Soccar], 16);
    assert_eq!(counts[&GameMode::Hoops], 12);
    assert_eq!(counts[&GameMode::Dropshot], 2);
    for mode in [GameMode::Soccar, GameMode::Hoops, GameMode::Dropshot] {
        let _arena = Arena::new(mode);
    }
    assert_eq!(
        rocketsim::get_arena_collision_mesh_files(GameMode::Heatseeker).len(),
        16
    );
    assert_eq!(
        rocketsim::get_arena_collision_mesh_files(GameMode::Snowday).len(),
        16
    );
}
