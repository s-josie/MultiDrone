from motion_planning import rrt_planner_mab, initialise
import random
import csv
import numpy as np
import os
from pathlib import Path




#want 15 problems (5 easy, 5 medium, 5 hard according to epsilon, alpha, beta criteria (calculate the values and provide them))
#bounds are fixed
#for each problem, provide different sets of initial and goal states such that I can test the same problem with K in [1,5] drones

#for each problem, # drones combo record the solving time (limit 1 min) or None if no solution found

#average across each difficulty level to get mean and 95% confidence interval

def test(nu_values: list[float], num_drones: list[int], env_path_list: list[str], time_limit: int, filename: str, save_dir: str = "results"):

    os.makedirs(save_dir, exist_ok=True)

    data = [] #initialise 

    for env in env_path_list:
        for K in num_drones:
            for nu in nu_values:

                #choose K drones only #TODO
            
                sim = initialise(n_drones=K, env_file=env)
                
                if sim.is_valid(sim.initial_configuration):
                    return ValueError("Invalid Environment")
                
                plan, solve_time = rrt_planner_mab(sim, nu, time_limit, env_file=env)


                difficulty = (Path(env).stem).split("_")[0]

                if plan == None:
                    row = [difficulty, env, K, nu, False, np.nan]
                else: 
                    row = [difficulty, env, K, nu, True, solve_time]

                data.append(row)

    save_path = os.path.join(save_dir, filename)
    with open(save_path, 'w', newline='', encoding='utf-8') as file:
        writer = csv.writer(file)
    
        # Write the header row
        writer.writerow(["Difficulty", "Environment Path", "Number of Drones", "Exploration Constant", "Solved (y/n)", "Solve Time"])
        
        # Write multiple data rows at once
        writer.writerows(data)

    #TODO make and save csv file with K on one axis, env (with env path as col name) o nthe other axis. the values in each cell is None (if no solution is found) or the time taken to find a solution (if it took less than 60 seconds)
    #make one .csv per nu value


    #make csv summarising success rate on hard, easy, medium envs


    return None


if __name__ == "__main__":

    random.seed(42) #for reproducability in experiments
    easy_env_path_list = ["motion_planning_workspaces/easy_01.yaml", "motion_planning_workspaces/easy_02.yaml", "motion_planning_workspaces/easy_03.yaml", "motion_planning_workspaces/easy_04.yaml", "motion_planning_workspaces/easy_05.yaml"]
    med_env_path_list = ["motion_planning_workspaces/medium_01.yaml", "motion_planning_workspaces/medium_02.yaml", "motion_planning_workspaces/medium_03.yaml", "motion_planning_workspaces/medium_04.yaml", "motion_planning_workspaces/medium_05.yaml"]
    hard_env_path_list = ["motion_planning_workspaces/hard_01.yaml", "motion_planning_workspaces/hard_02.yaml", "motion_planning_workspaces/hard_03.yaml", "motion_planning_workspaces/hard_04.yaml", "motion_planning_workspaces/hard_05.yaml"]
    nu_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.75]
    num_drones = [1,2,3,4,5]

    nu_envs = ["motion_planning_workspaces/easy_03.yaml", "motion_planning_workspaces/easy_05.yaml", "motion_planning_workspaces/medium_03.yaml", "motion_planning_workspaces/medium_05.yaml", "motion_planning_workspaces/hard_03.yaml", "motion_planning_workspaces/hard_05.yaml"]
    #first test nu values with 6 envs only, 20 sec time limit 3*3*6= 26 mins
    #test(nu_values, [1,3,5], nu_envs, 30)

    test([0.2, 0.3], [5], ["motion_planning_workspaces/easy_01.yaml", "motion_planning_workspaces/medium_02.yaml"], 20, "mini_test.csv")

    #using best nu value, test 15 envs, 1-5 num drones, 60 sec limit
    #test([best_nu], num_drones, env_path_list)