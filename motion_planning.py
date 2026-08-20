import numpy as np
from multi_drone import MultiDrone
import time
import yaml

def initialise(n_drones: int, env_file: str = "environment.yaml"):
    # Initialize the MultiDrone environment
    sim = MultiDrone(num_drones=n_drones, environment_file=env_file)

    # Obtain the initial configuration and the goal positions
    initial_configuration = sim.initial_configuration
    goal_positions = sim.goal_positions


    return sim, initial_configuration, goal_positions


def uniform_sampler(bounds: dict):

    rng = np.random.default_rng()

    low = [bounds["x"][0], bounds["y"][0], bounds["z"][0]]
    high = [bounds["x"][1], bounds["y"][1], bounds["z"][1]]
    
    point = rng.uniform(low, high)

    return point


"""def sample_near_obstacles(): #TODO




    return point

def expansive_space_sampler():


    return point


def multi_arm_bandit_sampler():

    return point"""


def collision_free_checker(point: tuple[float, float, float], obstacles: dict): #TODO

    if point :
        return True

    else:
        return False


def breadth_first_search(nodes, roadmap): #TODO

    

    return None

def my_planner(sim: MultiDrone, points_to_add: int, time_limit: int = 20, env_file: str = "environment.yaml"):

    #initialise
    print("starting clock")
    start_time = time.time()
    nodes = []
    roadmap = []

    with open(env_file, "r") as f:
        config = yaml.safe_load(f)

    while time.time() - start_time < time_limit:

        new_points = 0 #initialise 

        while new_points < points_to_add:

            #sample a configuration
            configuration = uniform_sampler(config["bounds"])

            #connect 
            if collision_free_checker(config["obstacles"]): #no collision
                #add configuration to roadmap
                nodes.append(configuration)
                new_points += 1
                
                #connect configuration to existing vertices in G using valid edges
                #
                for node in nodes:

                    if 

                    #check <= 30% distance of workspace between nodes
                    dist = 
                    if dist > 30: 
                        continue

                    #check valid path or not
                    if sim.motion_valid(start, end):
                        roadmap[i].append(node) #TODO
                
        #search g for path
        path = breadth_first_search(nodes, roadmap)
        if path != None:
            return path

    return None



# Once the MultiDrone environment is initialized,
# you can use it within a sampling-based motion planner, e.g.
'''
solution_path = my_planner(sim)
'''

# In the planner, you can use the following functions of the MultiDrone environment:

# 1.) Check if a configuration is valid
configuration = np.array([
    [5.0, 4.5, 3.0],
    [3.5, 10.0, 8.0]
], dtype=np.float32)
is_valid = sim.is_valid(configuration)
print(f"is valid: {is_valid}")

# 2.) Check if a straight-line motion between 'start' and 'end' is valid
start = np.array([
    [5.0, 4.5, 3.0],  # The start point of the first drone
    [3.5, 10.0, 8.0]  # The start point of the second drone 
], dtype=np.float32) 
end = np.array([
    [10.0, 20.0, 3.0],  # The end point of the first drone
    [3.5, 20.0, 15.0]  # The end point of the second drone 
], dtype=np.float32)
motion_valid = sim.motion_valid(start, end) 
print(f"motion valid: {motion_valid}")

# 3.) Check if a configuration reached the goal
configuration = np.array([
    [5.0, 4.5, 3.0],
    [3.5, 10.0, 8.0]
], dtype=np.float32)
goal_reached = sim.is_goal(configuration)
print(f"goal reached: {goal_reached}")

# 4.) Visualize a path
paths = [
    np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float32), # First waypoints
    np.array([[1, 1, 1], [2, 1, 1]], dtype=np.float32), # Second waypoints
    np.array([[2, 2, 2], [3, 2, 2]], dtype=np.float32), # Third waypoints
]
sim.visualize_paths(paths)