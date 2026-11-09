//! Optional roster metadata next to an RLPR recording.
//!
//! Read `<recording filename>.metadata.json`, for example
//! `x.rlpr.zst.metadata.json`. The file is advisory input, not recording
//! truth. It is used only after its SHA-256 matches the exact input bytes.

use std::{
    fs::{self, File},
    io::Read,
    path::{Path, PathBuf},
};

use rocketsim::Team;
use serde::Deserialize;
use sha2::{Digest, Sha256};

use super::common;

/// Metadata schema accepted by this tool.
pub const SCHEMA_VERSION: u32 = 1;

#[derive(Debug, Deserialize)]
struct MetadataFile {
    schema_version: u32,
    recording_sha256: String,
    car_bodies: Vec<String>,
    car_teams: Vec<String>,
}

/// Validated roster metadata for one recording.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LoadedMetadata {
    /// Body preset index per car slot, in [`common::BODY_PRESET_NAMES`] order.
    pub car_bodies: Vec<usize>,
    /// Team per car slot, frozen recording order.
    pub car_teams: Vec<Team>,
    /// Path of the metadata file that was read.
    pub path: PathBuf,
}

/// Adjacent metadata path: append `.metadata.json` to the input filename.
pub fn metadata_path(recording: &Path) -> PathBuf {
    let mut name = recording.as_os_str().to_os_string();
    name.push(".metadata.json");
    PathBuf::from(name)
}

/// Read and validate metadata for one recording.
///
/// Returns `None` when the metadata file is absent. A present file must pass
/// every check: schema version, exact input-file SHA-256, car-slot count,
/// body names, and team names. A failure is an error, never a silent fallback.
pub fn load(recording: &Path, car_slots: usize) -> Result<Option<LoadedMetadata>, String> {
    let path = metadata_path(recording);
    if !path.is_file() {
        return Ok(None);
    }

    let text = fs::read_to_string(&path)
        .map_err(|err| format!("{}: cannot read metadata: {err}", path.display()))?;
    let metadata: MetadataFile = serde_json::from_str(&text)
        .map_err(|err| format!("{}: invalid JSON metadata: {err}", path.display()))?;
    if metadata.schema_version != SCHEMA_VERSION {
        return Err(format!(
            "{}: unsupported metadata schema_version {}; expected {SCHEMA_VERSION}",
            path.display(),
            metadata.schema_version
        ));
    }

    let actual_sha = file_sha256(recording)
        .map_err(|err| format!("{}: cannot hash recording: {err}", recording.display()))?;
    if !actual_sha.eq_ignore_ascii_case(metadata.recording_sha256.trim()) {
        return Err(format!(
            "{}: recording_sha256 mismatch for {}; metadata says {}, file is {}",
            path.display(),
            recording.display(),
            metadata.recording_sha256,
            actual_sha
        ));
    }

    if metadata.car_bodies.len() != car_slots || metadata.car_teams.len() != car_slots {
        return Err(format!(
            "{}: metadata has {} bodies and {} teams for {car_slots} car slots",
            path.display(),
            metadata.car_bodies.len(),
            metadata.car_teams.len()
        ));
    }

    let car_bodies = metadata
        .car_bodies
        .iter()
        .enumerate()
        .map(|(slot, name)| parse_body_name(name, slot, &path))
        .collect::<Result<Vec<_>, _>>()?;
    let car_teams = metadata
        .car_teams
        .iter()
        .enumerate()
        .map(|(slot, name)| {
            parse_team(name).map_err(|err| format!("{}: car slot {slot}: {err}", path.display()))
        })
        .collect::<Result<Vec<_>, _>>()?;

    Ok(Some(LoadedMetadata {
        car_bodies,
        car_teams,
        path,
    }))
}

/// Parse a comma-separated team list such as `blue,orange,orange,blue`.
pub fn parse_teams(value: &str) -> Result<Vec<Team>, String> {
    value.split(',').map(parse_team).collect()
}

/// Parse one team name. `blue`/`0` and `orange`/`1` are accepted.
pub fn parse_team(value: &str) -> Result<Team, String> {
    match value.trim().to_ascii_lowercase().as_str() {
        "blue" | "0" => Ok(Team::Blue),
        "orange" | "1" => Ok(Team::Orange),
        other => Err(format!(
            "unknown team {other:?}; expected blue/orange or 0/1"
        )),
    }
}

/// Stable display name for one team.
pub fn team_name(team: Team) -> &'static str {
    match team {
        Team::Blue => "blue",
        Team::Orange => "orange",
    }
}

/// Display names for a team roster.
pub fn team_names(teams: &[Team]) -> Vec<&'static str> {
    teams.iter().copied().map(team_name).collect()
}

/// Old fallback: Blue first, then alternating by frozen car slot.
pub fn alternating_teams(count: usize) -> Vec<Team> {
    (0..count)
        .map(|slot| {
            if slot.is_multiple_of(2) {
                Team::Blue
            } else {
                Team::Orange
            }
        })
        .collect()
}

/// Stream the exact bytes of one input file through SHA-256.
///
/// A `.rlpr.zst` file hashes the compressed bytes. A plain `.rlpr` file hashes
/// the raw bytes. The loader never decompresses before hashing.
pub fn file_sha256(path: &Path) -> Result<String, String> {
    let mut file = File::open(path).map_err(|err| format!("{}: {err}", path.display()))?;
    let mut hasher = Sha256::new();
    let mut buffer = [0u8; 64 * 1024];
    loop {
        let read = file
            .read(&mut buffer)
            .map_err(|err| format!("{}: {err}", path.display()))?;
        if read == 0 {
            break;
        }
        hasher.update(&buffer[..read]);
    }
    Ok(format!("{:x}", hasher.finalize()))
}

fn parse_body_name(name: &str, slot: usize, path: &Path) -> Result<usize, String> {
    common::BODY_PRESET_NAMES
        .iter()
        .position(|preset| preset.eq_ignore_ascii_case(name.trim()))
        .ok_or_else(|| {
            format!(
                "{}: car slot {slot}: unknown body preset {name:?}; expected one of {}",
                path.display(),
                common::BODY_PRESET_NAMES.join(", ")
            )
        })
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use serde_json::json;

    use super::*;

    fn temp_dir(name: &str) -> PathBuf {
        let path = std::env::temp_dir().join(format!("rlpr-meta-{}-{name}", std::process::id()));
        let _ = fs::remove_dir_all(&path);
        fs::create_dir_all(&path).expect("create test directory");
        path
    }

    fn write_metadata(
        recording: &Path,
        sha: &str,
        bodies: &[&str],
        teams: &[&str],
        schema: u32,
    ) -> PathBuf {
        let path = metadata_path(recording);
        let text = json!({
            "schema_version": schema,
            "recording_sha256": sha,
            "car_bodies": bodies,
            "car_teams": teams,
            "provenance": {"test": "ignored"}
        });
        fs::write(&path, serde_json::to_vec(&text).unwrap()).expect("write metadata");
        path
    }

    #[test]
    fn reads_raw_and_zst_inputs_with_exact_byte_hash() {
        let dir = temp_dir("raw-zst");

        let raw = dir.join("a.rlpr");
        fs::write(&raw, b"raw recording bytes").unwrap();
        let raw_sha = file_sha256(&raw).unwrap();
        write_metadata(
            &raw,
            &raw_sha,
            &["Plank", "Plank"],
            &["blue", "orange"],
            SCHEMA_VERSION,
        );
        let loaded = load(&raw, 2).unwrap().unwrap();
        assert_eq!(loaded.car_bodies, vec![2, 2]);
        assert_eq!(loaded.car_teams, vec![Team::Blue, Team::Orange]);

        let zst = dir.join("b.rlpr.zst");
        fs::write(&zst, b"compressed bytes, not decompressed").unwrap();
        let zst_sha = file_sha256(&zst).unwrap();
        write_metadata(
            &zst,
            &zst_sha,
            &["Octane", "Merc"],
            &["orange", "blue"],
            SCHEMA_VERSION,
        );
        let loaded = load(&zst, 2).unwrap().unwrap();
        assert_eq!(loaded.car_bodies, vec![0, 5]);
        assert_eq!(loaded.car_teams, vec![Team::Orange, Team::Blue]);

        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn rejects_stale_hash_bad_names_and_bad_row_count() {
        let dir = temp_dir("reject");
        let recording = dir.join("x.rlpr");
        fs::write(&recording, b"recording").unwrap();
        let sha = file_sha256(&recording).unwrap();

        write_metadata(
            &recording,
            &"0".repeat(64),
            &["Octane"],
            &["blue"],
            SCHEMA_VERSION,
        );
        let error = load(&recording, 1).unwrap_err();
        assert!(error.contains("recording_sha256 mismatch"), "{error}");

        write_metadata(&recording, &sha, &["Octane"], &["blue"], SCHEMA_VERSION);
        assert!(load(&recording, 2).unwrap_err().contains("2 car slots"));

        write_metadata(&recording, &sha, &["Unknown"], &["blue"], SCHEMA_VERSION);
        assert!(
            load(&recording, 1)
                .unwrap_err()
                .contains("unknown body preset")
        );

        write_metadata(&recording, &sha, &["Octane"], &["purple"], SCHEMA_VERSION);
        assert!(load(&recording, 1).unwrap_err().contains("unknown team"));

        fs::remove_dir_all(dir).unwrap();
    }

    #[test]
    fn absent_metadata_returns_none_and_fallback_is_alternating() {
        let dir = temp_dir("absent");
        let recording = dir.join("x.rlpr");
        fs::write(&recording, b"recording").unwrap();
        assert!(load(&recording, 2).unwrap().is_none());
        assert_eq!(
            alternating_teams(4),
            vec![Team::Blue, Team::Orange, Team::Blue, Team::Orange]
        );
        assert_eq!(
            parse_teams("blue,1,ORANGE,0").unwrap(),
            vec![Team::Blue, Team::Orange, Team::Orange, Team::Blue]
        );
        assert_eq!(team_names(&alternating_teams(2)), vec!["blue", "orange"]);
        fs::remove_dir_all(dir).unwrap();
    }
}
