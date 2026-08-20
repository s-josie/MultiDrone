import numpy as np
from multi_drone import MultiDrone
import time
import yaml
from collections import deque
import random


#Initialise environment:
def initialise(n_drones: int, env_file: str = "environment.yaml"):
    # Initialize the MultiDrone environment
    sim = MultiDrone(num_drones=n_drones, environment_file=env_file)

    return sim


#Sampling functions: 
def uniform_sampler(bounds: dict, n_drones: int):

    rng = np.random.default_rng()

    low = [bounds["x"][0], bounds["y"][0], bounds["z"][0]]
    high = [bounds["x"][1], bounds["y"][1], bounds["z"][1]]
    
    point = rng.uniform(low, high, size = (n_drones, 3))

    return point


"""def sample_near_obstacles(): #TODO


    return point

def expansive_space_sampler():


    return point


def multi_arm_bandit_sampler():

    return point
    
    
def goal_sampler():

    return point"""


#Graph search functions:
def breadth_first_search(nodes: list, roadmap: list, sim: MultiDrone):

    #initialise 
    queue = deque([0]) #frontier (discoverd but not explored)
    visited = {0} #set stores visited nodes to prevent looping in path
    parent = {0: None} #dict that stores {node_id, parent node} to create path

    while queue: #still nodes to explore

        current = queue.popleft() #First In First Out for BFS

        #check whether configuration satisfies the goal
        if sim.is_goal(nodes[current]):

            #reconstruct path
            path = []
            node = current

            while node is not None:
                path.append(nodes[node]) #add corresponding configuration to path
                node = parent[node] #update node to move up path

            return path[::-1] #reverses path to go from initial to goal state

        #expand neighbours
        for neighbour in roadmap[current]:

            if neighbour not in visited: #don't expand same node again to avoid loops
                visited.add(neighbour) 
                parent[neighbour] = current #record parent for neighbour
                queue.append(neighbour) #add to queue

    return None



#my planner:
def my_planner(sim: MultiDrone, points_to_add: int, time_limit: int = 20, env_file: str = "environment.yaml"):

    #initialise
    print("starting clock")
    start_time = time.time()
    nodes = [sim.initial_configuration]
    roadmap = [[]]

    with open(env_file, "r") as f:
            config = yaml.safe_load(f)

    #calculate 30% diagnoal distance across workspace in each direction for use in connection step
    workspace_diagonal = np.linalg.norm(np.array([config["bounds"]["x"][1] - config["bounds"]["x"][0], config["bounds"]["y"][1] - config["bounds"]["y"][0], config["bounds"]["z"][1] - config["bounds"]["z"][0]]))
    max_connection_distance = 0.3 * workspace_diagonal

    while time.time() - start_time < time_limit:

        new_points = 0 #initialise 

        while new_points < points_to_add:

            #sample a configuration
            configuration = uniform_sampler(config["bounds"], n_drones=len(config["initial_configuration"]))

            #connect 
            if sim.is_valid(configuration): #no collision

                #get node_id
                node_id = len(nodes)

                #add configuration to roadmap
                nodes.append(configuration)
                roadmap.append([]) #to store neighbours to this node
                new_points += 1
                
                #connect configuration to existing vertices in G using valid edges
                for neighbour_id, potential_connection in enumerate(nodes[:-1]): #exclude the most recent node added to avoid comparison with self

                    #check <= 30% distance of workspace between nodes to be connected
                    distances = np.linalg.norm(potential_connection - configuration, axis=1)

                    if np.any(distances > max_connection_distance): 
                        continue

                    #check valid path or not
                    if sim.motion_valid(configuration, potential_connection):
                        #add path each way to roadmap 
                        roadmap[node_id].append(neighbour_id)
                        roadmap[neighbour_id].append(node_id)
                
        #search g for path
        path = breadth_first_search(nodes, roadmap, sim)
        if path != None:
            return path

    print("time up!")
    return None






if __name__ == "__main__":
    random.seed(42) #for reproducability in experiments
    sim = initialise(n_drones=2, env_file="environment.yaml")
    solution_path = my_planner(sim, points_to_add=10)
    sim.visualize_paths(solution_path)
