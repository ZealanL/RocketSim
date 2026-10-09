//! Detached 3D visualizer for RocketSim (v2 rebuild).
//!
//! The sim streams frames over TCP to the [`vis_viewer`](crate::bin) process;
//! see `tools/README.md` for the screenshot harness and stage plan.
//!
//! Rebuild stages (each gated on tests + screenshot + perf):
//! * 0 — harness + deterministic scene ([`photo`]) ✅
//! * 1 — empty window + overlay (current)
//! * 2 — wire protocol (`proto` + `NetVis`)
//! * 3+ — triangle → arena → cars → pads → lines → cameras → menus → FX.

pub mod photo;
