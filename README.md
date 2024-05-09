![image](https://user-images.githubusercontent.com/36944229/219303954-7267bce1-b7c5-4f15-881c-b9545512e65b.png)

**A C++ library for simulating Rocket League games at maximum efficiency**

RocketSim is a complete simulation of Rocket League's gameplay logic and physics that is completely standalone.
RocketSim supports the game modes: Soccar, Hoops, Heatseeker, and Snowday.

# Speed
RocketSim is designed to run extremely fast, even when complex collisions and suspension calculations are happening every tick.
On an average PC running a single thread of RocketSim with two cars, RocketSim can simulate around 20 minutes of game time every second.
This means that with 12 threads running RocketSim, you can simulate around 10 days of game time every minute!

# Accuracy
RocketSim is not a perfectly accurate replication of Rocket League, but is close enough for most applications (such as training ML bots).
Perceivable differences between the simulation and the real game usually take at least a second to accumulate from an initial state.
This means RocketSim is accurate enough to:
- *Train machine learning bots*
- *Simulate different shots on the ball at different angles to find the best input combination*
- *Simulate air control to find the optimal orientation input*
- *Simulate ground and floor pinches*

However, RocketSim is NOT accurate enough to:
- *Accurately re-create entire games from inputs alone*
- *Perfectly simulate long sequences of jumps and landings*

## Installation
- Clone this repo and build it
- Use https://github.com/ZealanL/RLArenaCollisionDumper to dump all of Rocket League's arena collision meshes
- Move those assets into RocketSim's executing directory

## Documentation
Documentation is available at: https://zealanl.github.io/RocketSimDocs/

## Bindings
If you don't want to work in C++, here are some (unofficial) bindings written in other languages:
- **Python**: https://github.com/mtheall/RocketSim by `mtheall`
- **Python**: https://github.com/uservar/pyrocketsim by `uservar`
- **Rust**: https://github.com/VirxEC/rocketsim-rs by `VirxEC`

Official Python bindings are currently in the works.

## Performance Details
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
v2.0.0 = ~50,000tps
v2.1.0 = 114,481tps
```

## Issues & PRs
Feel free to make issues and pull requests if you encounter any issues!

You can also contact me on Discord if you have questions: `Zealan#5987`

## Legal Notice
RocketSim was written to replicate Rocket League's game logic, but does not actually contain any code from the game.
To Epic Games/Psyonix: If any of you guys have an issue with this, let me know on Discord and we can resolve it.
