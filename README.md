![image](https://user-images.githubusercontent.com/36944229/219303954-7267bce1-b7c5-4f15-881c-b9545512e65b.png)

**A C++ library for simulating Rocket League games at maximum efficiency**

## Installation
- Clone this repo and build it
- Use https://github.com/ZealanL/RLArenaCollisionDumper to dump all of Rocket League's arena collision meshes
- Move those assets into RocketSim's executing directory

## Progress
**Coming Soon:**
- Significant collision optimizations
- Proper documentation

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

## Simulation Accuracy
RocketSim is not perfectly accurate, but it's close enough that it shouldnt matter (for ML bots or humans).
Bots that work well in RocketSim will work well in the actual game, and visa-versa.

## Example Usage
```cpp
#include "./RocketSim/src/Sim/Arena/Arena.h"

// Make an arena instance (this is where our simulation takes place, has its own btDynamicsWorld instance)
Arena* arena = Arena::Create(GameMode::SOCCAR);

// Make a new car
// NOTE: The ball and all cars are freed from memory when their arena is deconstructed, you don't need to do it yourself
Car* car = arena->AddCar(Team::BLUE);

// Set up an initial state for our car
CarState carState = {};
carState.pos = { 0.f, 0.f, 17.f };
carState.vel = { 50.f, 0.f, 0.f };
car->SetState(carState);

// Setup a ball state
BallState ballState = {};
ballState.pos = { 0.f, 400.f, 100.f };
arena->ball->SetState(ballState);

// Make our car drive forward and turn
car->controls.throttle = 1;
car->controls.steer = 1;

// Simulate for 100 ticks
arena->Step(100);

// Lets see where our car went!
std::cout << "After " << arena->tickCount << "ticks, our car is at: " << car->GetState().pos << std::endl;

// Destroy the arena once we are done
delete arena;
```

## Issues & PRs
Feel free to make issues and pull requests if you encounter any issues!

You can also contact me on Discord if you have questions: `Zealan#5987`

## Legal Notice
RocketSim was written to replicate Rocket League's game logic, but does not actually contain any code from the game.
To Epic Games/Psyonix: If any of you guys have an issue with this, let me know on Discord and we can resolve it.
