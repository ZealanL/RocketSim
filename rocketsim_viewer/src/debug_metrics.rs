//! Optional bounded timing samples. GPU timings use asynchronous elapsed-time queries.

use std::{
    path::PathBuf,
    time::{Duration, Instant},
};

use crate::render::debug::DebugStats;

pub struct DebugMetrics {
    path: PathBuf,
    started: Instant,
    flushed: Instant,
    cpu: Vec<f64>,
    draw: Vec<f64>,
    debug_cpu: Vec<f64>,
    gpu: Vec<f64>,
}

impl DebugMetrics {
    pub fn new(path: PathBuf) -> Self {
        Self {
            path,
            started: Instant::now(),
            flushed: Instant::now(),
            cpu: Vec::new(),
            draw: Vec::new(),
            debug_cpu: Vec::new(),
            gpu: Vec::new(),
        }
    }

    pub fn record(
        &mut self,
        cpu: f64,
        draw: f64,
        fps: f64,
        stats: DebugStats,
        groups: usize,
        transfers: u64,
    ) {
        if self.started.elapsed() < Duration::from_secs(2) || self.cpu.len() >= 10_000 {
            return;
        }
        self.cpu.push(cpu);
        self.draw.push(draw);
        self.debug_cpu.push(stats.cpu_ms);
        if let Some(gpu) = stats.gpu_ms {
            self.gpu.push(gpu);
        }
        if self.flushed.elapsed() < Duration::from_millis(500) {
            return;
        }
        self.flushed = Instant::now();
        let text = format!(
            "{{\"samples\":{},\"warmup_seconds\":2,\"fps\":{:.2},\"cpu_ms_p50\":{:.4},\"cpu_ms_p95\":{:.4},\"draw_ms_p50\":{:.4},\"draw_ms_p95\":{:.4},\"debug_cpu_ms_p50\":{:.4},\"debug_cpu_ms_p95\":{:.4},\"debug_gpu_ms_p50\":{:.4},\"debug_gpu_ms_p95\":{:.4},\"gpu_samples\":{},\"groups\":{},\"draw_calls\":{},\"segments\":{},\"points\":{},\"instances\":{},\"triangles\":{},\"mesh_uploads\":{},\"line_uploads\":{},\"texture_uploads\":{},\"upload_bytes\":{},\"debug_transfers\":{}}}\n",
            self.cpu.len(),
            fps,
            percentile(&self.cpu, 0.5),
            percentile(&self.cpu, 0.95),
            percentile(&self.draw, 0.5),
            percentile(&self.draw, 0.95),
            percentile(&self.debug_cpu, 0.5),
            percentile(&self.debug_cpu, 0.95),
            percentile(&self.gpu, 0.5),
            percentile(&self.gpu, 0.95),
            self.gpu.len(),
            groups,
            stats.draw_calls,
            stats.segments,
            stats.points,
            stats.instances,
            stats.primitives,
            stats.mesh_uploads,
            stats.line_uploads,
            stats.texture_uploads,
            stats.upload_bytes,
            transfers,
        );
        if let Err(error) = std::fs::write(&self.path, text) {
            eprintln!("rocketsim-viewer: cannot write debug statistics: {error}");
        }
    }
}

fn percentile(values: &[f64], fraction: f64) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    let mut sorted = values.to_vec();
    sorted.sort_unstable_by(f64::total_cmp);
    sorted[((sorted.len() - 1) as f64 * fraction).round() as usize]
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn percentiles_handle_empty_and_unsorted_samples() {
        assert_eq!(percentile(&[], 0.5), 0.0);
        assert_eq!(percentile(&[5.0, 1.0, 3.0], 0.5), 3.0);
        assert_eq!(percentile(&[5.0, 1.0, 3.0], 0.95), 5.0);
    }
}
