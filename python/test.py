import rocketsim
from rocketsim import Arena, Car, CarConfig, CarState, Ball, BallState, GameMode, Team, Vec

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

# Save current arena state to a file
file_path = "test_arena_file.bin"
arena.serialize_to_file(file_path)

# Load that file into a new arena
new_arena = Arena.deserialize_new_from_file(file_path)
