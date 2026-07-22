![A logo saying RocketSim](https://user-images.githubusercontent.com/36944229/219303954-7267bce1-b7c5-4f15-881c-b9545512e65b.png)

**A Rust library for simulating Rocket League games at maximum efficiency**

A port of <https://github.com/ZealanL/RocketSim>. All the credit goes to Zealan for doing the hard work.

## What is this?

This repo is my attempt and simplifying and optimizing RocketSim.
It does not care for Bullet compatibility, and removes as many abstrations as possible.

This project is currently a massive WIP, and highly experimental.
Lots of features found in RocketSim are currently missing.

## Arena memory modes

`ArenaConfig::mem_weight_mode` controls the broadphase memory/performance tradeoff. In the `stress_v3` benchmark:

| Mode | 1v1 memory per arena | 1v1 performance | 3v3 memory per arena | 3v3 performance |
| --- | ---: | ---: | ---: | ---: |
| `Heavy` (default) | ~915 KiB | Baseline | ~946 KiB | Baseline |
| `Balanced` | ~108 KiB | ~1–2% slower | ~123 KiB | ~3% slower |
| `Light` | ~66 KiB | ~9% slower | ~91 KiB | ~12% slower |

Balanced reduces marginal memory while retaining most of Heavy's performance. Light effectively disables grid partitioning by using a single broadphase cell, prioritizing minimum memory over performance. Results may vary by platform and workload.
