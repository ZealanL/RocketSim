# RocketSim
 A C++ library for simulating Rocket League games at maximum efficiency

## Accuracy
RocketSim is not perfectly accurate, but it's close enough that it shouldnt matter.
Bots that work well in RocketSim will work well in the actual game, and visa-versa.

## Example Usage
```cpp
#include "./RocketSim/src/Sim/Arena/Arena.h"

// Make an arena instance (this is where our simulation takes place, has its own btDynamicsWorld instance)
Arena arena = Arena(GameMode::SOCCAR);

// Make a new car
// NOTE: The ball and all cars are freed from memory when their arena is deconstructed, you don't need to do it yourself
Car* car = arena.AddCar(Team::BLUE);

// Set up an initial state for our car
CarState setupState = {};
carState.pos = { 0.f, 0.f, 17.f };
carState.vel = { 50.f, 0.f, 0.f };
car->SetState(setupState);

// Setup a ball state
BallState ballState = {};
ballState.pos = { 0.f, 400.f, 100.f }
arena.ball->SetState(ballState);

// Make our car drive forward and turn
car->controls.throttle = 1;
car->controls.boost = 1;

// Simulate for 100 ticks
arena.Step(100);

// Lets see where our car went!
std::cout << "After " << arena.tickCount << "ticks, our car is at: " << car->GetState().pos << std::endl;
```
