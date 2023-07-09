![image](https://user-images.githubusercontent.com/36944229/219303954-7267bce1-b7c5-4f15-881c-b9545512e65b.png)

**A C++ library for simulating Rocket League games at maximum efficiency**

## Installation
- Clone this repo and build it
- Use https://github.com/ZealanL/RLArenaCollisionDumper to dump all of Rocket League's arena collision meshes
- Move those assets into RocketSim's executing directory

## Documentation
Documentation is available at: https://zealanl.github.io/RocketSimDocs/

## Progress
**Coming Soon:**
- More collision optimizations
- Proper documentation
- Improved collision accuracy

**Done:**
- Car suspension
- Car driving
- Car jumps and flips
- Arena collision
- Proper ball bounces
- Car-ball collision with proper forces (will be refined more in the near future)
- Boost usage and boost pads
- Bumps and demos
- Auto-flip when upside-down
- Serialization of cars/ball/boost pads/arena
- Boost pad/suspension ray optimization using lookup grid

## Bindings
If you don't want to work in C++, here are some (unofficial) bindings written in other languages:
- **Python**: https://github.com/mtheall/RocketSim by `mtheall`
- **Python**: https://github.com/uservar/pyrocketsim by `uservar`
- **Rust**: https://github.com/VirxEC/rocketsim-rs by `VirxEC`

Official Python bindings are currently in the works.

## Performance
RocketSim already heavily outperforms the speed of Rocket League's physics tick step without optimization.

Version performance comparison:
```
OS: Windows 10 (Process Priority = Normal)
CPU: Intel i5-11400 @ 2.60GHz
Ram Speed: 3200MZ
Compiler: MSVC 14.16
=================================
Arena: Default (Soccar)
Cars: 2 on each team (2v2)
Inputs: Randomly pre-generated, changed every 2-60 ticks for each car
=================================
Single-thread performance (calculated using average CPU cycles per tick on the RocketSim thread) (1M ticks simulated):
v1.0.0 = 30,334tps
v1.1.0 = 48,191tps
v1.2.0 = 50,763tps
```

## Simulation Accuracy
RocketSim is not perfectly accurate, but it's close enough that it shouldn't matter (for ML bots or humans).
Bots that work well in RocketSim will work well in the actual game and visa-versa.

## Issues & PRs
Feel free to make issues and pull requests if you encounter any issues!

You can also contact me on Discord if you have questions: `Zealan#5987`

## Legal Notice
RocketSim was written to replicate Rocket League's game logic, but does not actually contain any code from the game.
To Epic Games/Psyonix: If any of you guys have an issue with this, let me know on Discord and we can resolve it.
