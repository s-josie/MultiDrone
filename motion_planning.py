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


def sample_near_configuration(q1: list, maximum_connection_dist: float, bounds: dict):
    q1 = np.asarray(q1, dtype=np.float32)
    q2 = np.empty_like(q1)

    lower = np.array([bounds["x"][0], bounds["y"][0], bounds["z"][0]])

    upper = np.array([bounds["x"][1], bounds["y"][1], bounds["z"][1]])

    for i in range(q1.shape[0]):

        #random direction in 3D, using gaussian for sphere
        direction = np.random.normal(size=3)
        direction /= np.linalg.norm(direction) #normalise to get unit vector

        #sample random uniform distance within sphere according to volumne TODO
        distance = (maximum_connection_dist* np.random.random() ** (1 / 3))

        q2[i] = q1[i] + distance * direction #vector addition by component

    # Reject if outside workspace
    if np.any(q2 < lower) or np.any(q2 > upper):
        return None

    return q2


def bridge_sampling(sim: MultiDrone, bounds:dict, n_drones: int, maximum_connection_dist: float):

    q1 = uniform_sampler(bounds, n_drones)

    #sample uniformly at random from the set of all configurations within maximum_connection_dist from q1
    q2 = sample_near_configuration(q1, maximum_connection_dist, bounds)

    if q2 is None:
        return None

    if sim.is_valid(q1) and not sim.is_valid(q2): #sampling near obstacle
        return q1
    elif sim.is_valid(q2) and not sim.is_valid(q1): #sampling near obstacle
        return q2
    elif not sim.is_valid(q1) and not sim.is_valid(q2): #sampling inside passage
        qm = 0.5*(q1+q2) 
        if sim.is_valid(qm):
            return qm
    else:
        return None


def goal_sampler(sim: MultiDrone):

    return sim.goal_positions.copy()

def multi_arm_bandit_sampler(sim, config:dict, max_connection_distance: float, bandit_weights: list[float], nu:float = 0.5):

    #calculate prob dist
    probs = list(map(lambda x: (1-nu)*(x/sum(bandit_weights))+nu/3, bandit_weights)) #TODO three cause we have three samplers

    #print(probs)

    #choose sampler according to prob dist
    sampler = random.choices(["uniform", "bridge", "goal"], probs)[0]

    if sampler == "uniform":
        return uniform_sampler(config["bounds"], sim.N), 0, probs[0]
    elif sampler == "bridge":
        return bridge_sampling(sim, config["bounds"], sim.N, max_connection_distance), 1, probs[1]
    else: #sampler == "goal"
        return goal_sampler(sim), 2, probs[2]

def bandit_weight_update(bandit_weights: list[float], reward: float, sampler_id: int, nu: float, prob_i: float):
    bandit_weights[sampler_id] = bandit_weights[sampler_id]*np.exp(nu*(reward/prob_i)/3) #div 3 for three sampling methods
    return None

def rrt_planner_mab(sim: MultiDrone, nu: float, time_limit: int = 20, env_file: str = "environment.yaml"):

    # initialise
    print("starting clock")
    start_time = time.time()
    bandit_weights = [1.0, 1.0, 1.0] #uniform, obstacle, goal

    with open(env_file, "r") as f:
        config = yaml.safe_load(f)

    # Maximum distance that a new node can be from its parent
    workspace_diagonal = np.linalg.norm(
        np.array([
            config["bounds"]["x"][1] - config["bounds"]["x"][0],
            config["bounds"]["y"][1] - config["bounds"]["y"][0],
            config["bounds"]["z"][1] - config["bounds"]["z"][0]
        ])
    )

    max_connection_distance = 0.3 * workspace_diagonal

    #set the initial configuration as the root node
    nodes = [sim.initial_configuration.copy()]
    parent = {0: None}


    while time.time() - start_time < time_limit:

        sample_config, i, prob_i = multi_arm_bandit_sampler(sim, config, max_connection_distance, bandit_weights, nu)

        if sample_config is None: 
            reward = 0 
            bandit_weight_update(bandit_weights, reward, i, nu, prob_i)

            continue

        #find id of nearest node in existing tree
        #TODO edit distance calc
        nearest_id = min(range(len(nodes)), key=lambda i: np.linalg.norm(nodes[i] - sample_config))

        nearest_node = nodes[nearest_id]

        #find distance between nearest node and sample_config
        direction = sample_config - nearest_node
        distance = np.linalg.norm(direction)

        #avoid division by zero
        if distance < 1e-6:
            reward = 0
            bandit_weight_update(bandit_weights, reward, i, nu, prob_i)

            continue

        #limit the extension to max_connection_distance
        step = min(distance, max_connection_distance)

        new_node = nearest_node + (direction / distance) * step


        #check that new_node is a valid config
        if not sim.is_valid(new_node):
            reward = 0
            bandit_weight_update(bandit_weights, reward, i, nu, prob_i)
            continue

        #check there is a valid path from nearest node to new_node
        if not sim.motion_valid(nearest_node, new_node):
            reward = 0
            bandit_weight_update(bandit_weights, reward, i, nu, prob_i)
            continue

        #calculate reward function if node is added
        #TODO make reward more informative about distance to goal
        reward = 1
        #update weights
        bandit_weight_update(bandit_weights, reward, i, nu, prob_i)

        #add new node to tree
        new_node_id = len(nodes)
        nodes.append(new_node)
        parent[new_node_id] = nearest_id #parent is nearest node already in tree

        #check if goal is reached
        if sim.is_goal(new_node):

            path = []
            current = new_node_id

            while current is not None: #move through parent dict to get path
                path.append(nodes[current])
                current = parent[current]

            #print(time.time()-start_time)
            return path[::-1] #reverse list so it goes from initial to goal state

    #print(time.time()-start_time)
    return None




def rrt_planner_mab_for_testing(sim: MultiDrone, nu: float, time_limit: int = 20, env_file: str = "environment.yaml"):

    # initialise
    print("starting clock")
    start_time = time.time()
    bandit_weights = [1.0, 1.0, 1.0] #uniform, obstacle, goal

    with open(env_file, "r") as f:
        config = yaml.safe_load(f)

    # Maximum distance that a new node can be from its parent
    workspace_diagonal = np.linalg.norm(
        np.array([
            config["bounds"]["x"][1] - config["bounds"]["x"][0],
            config["bounds"]["y"][1] - config["bounds"]["y"][0],
            config["bounds"]["z"][1] - config["bounds"]["z"][0]
        ])
    )

    max_connection_distance = 0.3 * workspace_diagonal

    #set the initial configuration as the root node
    nodes = [sim.initial_configuration.copy()]
    parent = {0: None}


    while time.time() - start_time < time_limit:

        sample_config, i, prob_i = multi_arm_bandit_sampler(sim, config, max_connection_distance, bandit_weights, nu)

        if sample_config is None: 
            reward = 0 
            bandit_weight_update(bandit_weights, reward, i, nu, prob_i)

            continue

        #find id of nearest node in existing tree
        #TODO edit distance calc
        nearest_id = min(range(len(nodes)), key=lambda i: np.linalg.norm(nodes[i] - sample_config))

        nearest_node = nodes[nearest_id]

        #find distance between nearest node and sample_config
        direction = sample_config - nearest_node
        distance = np.linalg.norm(direction)

        #avoid division by zero
        if distance < 1e-6:
            reward = 0
            bandit_weight_update(bandit_weights, reward, i, nu, prob_i)

            continue

        #limit the extension to max_connection_distance
        step = min(distance, max_connection_distance)

        new_node = nearest_node + (direction / distance) * step


        #check that new_node is a valid config
        if not sim.is_valid(new_node):
            reward = 0
            bandit_weight_update(bandit_weights, reward, i, nu, prob_i)
            continue

        #check there is a valid path from nearest node to new_node
        if not sim.motion_valid(nearest_node, new_node):
            reward = 0
            bandit_weight_update(bandit_weights, reward, i, nu, prob_i)
            continue

        #calculate reward function if node is added
        #TODO make reward more informative about distance to goal
        reward = 1
        #update weights
        bandit_weight_update(bandit_weights, reward, i, nu, prob_i)

        #add new node to tree
        new_node_id = len(nodes)
        nodes.append(new_node)
        parent[new_node_id] = nearest_id #parent is nearest node already in tree

        #check if goal is reached
        if sim.is_goal(new_node):

            path = []
            current = new_node_id

            while current is not None: #move through parent dict to get path
                path.append(nodes[current])
                current = parent[current]

            #print(time.time()-start_time)
            return path[::-1], time.time()-start_time #TODO edit after testing #reverse list so it goes from initial to goal state

    #print(time.time()-start_time)
    return None, np.nan #TODO edit after testing



def rrt_planner(sim: MultiDrone, sampler: str, time_limit: int = 20, env_file: str = "environment.yaml"):

    # initialise
    print("starting clock")
    start_time = time.time()

    with open(env_file, "r") as f:
        config = yaml.safe_load(f)

    # Maximum distance that a new node can be from its parent
    workspace_diagonal = np.linalg.norm(
        np.array([
            config["bounds"]["x"][1] - config["bounds"]["x"][0],
            config["bounds"]["y"][1] - config["bounds"]["y"][0],
            config["bounds"]["z"][1] - config["bounds"]["z"][0]
        ])
    )

    max_connection_distance = 0.3 * workspace_diagonal

    #set the initial configuration as the root node
    nodes = [sim.initial_configuration.copy()]
    parent = {0: None}

    while time.time() - start_time < time_limit:

        #sampler = random.choice(["uniform", "bridge", "goal"])
        #sample a configuration according to strategy
        if sampler == "uniform":
            sample_config = uniform_sampler(config["bounds"], sim.N)
        elif sampler == "bridge":
            sample_config = bridge_sampling(sim, config["bounds"], sim.N, max_connection_distance)
        elif sampler == "goal":
            sample_config = goal_sampler(sim)
        else: 
            return ValueError("Need to provide a valid sampling strategy.")

        if sample_config is None: 
            continue

        #find id of nearest node in existing tree
        nearest_id = min(range(len(nodes)), key=lambda i: np.linalg.norm(nodes[i] - sample_config))

        nearest_node = nodes[nearest_id]

        #find distance between nearest node and sample_config
        direction = sample_config - nearest_node
        distance = np.linalg.norm(sample_config - nearest_node)

        #avoid division by zero
        if distance < 1e-6:
            continue

        #limit the extension to max_connection_distance
        step = min(distance, max_connection_distance)

        new_node = nearest_node + (direction / distance) * step

        #check that new_node is a valid config
        if not sim.is_valid(new_node):
            continue

        #check there is a valid path from nearest node to new_node
        if not sim.motion_valid(nearest_node, new_node):
            continue

        #add new node to tree
        new_node_id = len(nodes)
        nodes.append(new_node)
        parent[new_node_id] = nearest_id #parent is nearest node already in tree

        #check if goal is reached
        if sim.is_goal(new_node):

            path = []
            current = new_node_id

            while current is not None: #move through parent dict to get path
                path.append(nodes[current])
                current = parent[current]

            #print(time.time()-start_time)
            return path[::-1] #reverse list so it goes from initial to goal state

    return None



"""
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


def prm_planner(sim: MultiDrone, points_to_add: int, time_limit: int = 20, env_file: str = "environment.yaml"):

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
                #TODO could connect to k closest neighbours
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

def expansive_space_sampler(maximum_connection_dist: float, config: dict): #TODO

    return None

"""




if __name__ == "__main__":
    random.seed(42) #for reproducability in experiments

    env_file = "motion_planning_workspaces/hard_04.yaml"
    sim = initialise(n_drones=5, env_file=env_file)
    #print(sim.is_valid(sim.initial_configuration))

    solution_path, time = rrt_planner_mab(sim, 0.3, 20, env_file=env_file)
    #solution_path = rrt_planner(sim, "uniform")
    sim.visualize_paths(solution_path)
    #print(time)
