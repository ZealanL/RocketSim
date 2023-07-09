import rocketsim
from rocketsim import Arena, Car, CarConfig, CarState, Ball, BallState, GameMode, Team, Vec, DataStreamIn, DataStreamOut

rocketsim.init("../collision_meshes")

# Create arena
arena = Arena(GameMode.SOCCAR)

# Add car
car = arena.add_car(Team.BLUE, CarConfig.OCTANE)

# Set car state to facing towards ball, not too far away, with max boost
car_state = car.get_state()
car_state.pos = Vec(0, -500, 17)
car_state.boost = 100
car.set_state(car_state)

# Set car controls to throttle forward and boost
car.controls.throttle = 1
car.controls.boost = True

# Set goal score callback to reset to kickoff and print a message
def goal_scored_callback(arena, team):
	print("Goal was scored for team", team)
	arena.reset_to_random_kickoff()
	
arena.set_goal_scored_callback(goal_scored_callback);

# Simulate arena for enough time for goal to be scored
arena.step(400)

# Print state of ball and car
ball_state = arena.ball.get_state()
car_state = car.get_state()
print("After {} ticks, car is at: {}, ball is at: {}".format(arena.get_tickcount(), car_state.pos, ball_state.pos))

file_path = "_test_file.bin"
print("Serializing arena to \"" + file_path + "\"...")
ds_out = DataStreamOut()
arena.serialize(ds_out)
ds_out.write_to_file(file_path, True)

print("Deserializing new arena...")
ds_in = DataStreamIn.read_from_file(file_path, True)
new_arena = arena.deserialize_new(ds_in)

print("Stepping both arenas for 20 ticks...")
arena.step(20)
new_arena.step(20)

print(" - Original arena's car is at:\t", arena.get_cars()[0].get_state().pos)
print(" - New arena's car is at:\t", new_arena.get_cars()[0].get_state().pos)
