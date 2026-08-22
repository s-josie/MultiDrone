from motion_planning import rrt_planner_mab, initialise
import random
import csv
import numpy as np
import os
from pathlib import Path
import pandas as pd
import scipy




#want 15 problems (5 easy, 5 medium, 5 hard according to epsilon, alpha, beta criteria (calculate the values and provide them))
#bounds are fixed
#for each problem, provide different sets of initial and goal states such that I can test the same problem with K in [1,5] drones

#for each problem, # drones combo record the solving time (limit 1 min) or None if no solution found

#average across each difficulty level to get mean and 95% confidence interval

def test(nu_values: list[float], env_path_list: list[str], time_limit: int, filename: str, save_dir: str = "results"):

    os.makedirs(save_dir, exist_ok=True)

    data = [] #initialise 

    for env in env_path_list:
        for nu in nu_values:

            K = int(env.split("/")[1].split("=")[1])

            sim = initialise(n_drones=K, env_file=env)

            if not sim.is_valid(sim.initial_configuration):
                print(f"invalid env + {env}")
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
        writer.writerow(["Difficulty", "Environment Path", "Number of Drones", "Exploration Constant", "Solved (T/F)", "Solve Time"])
        
        # Write multiple data rows at once
        writer.writerows(data)

    return None


def summarise_nu(csv_path: str, filename: str, save_dir: str = "results"):

    df = pd.read_csv(csv_path) #read in nu experiment data

    summary = (
        df.groupby("Exploration Constant").agg( #group by nu value to assess dif nu values
            success_percentage = ("Solved (T/F)", lambda x: (x == True).mean() * 100), #get mean solve percentage
            mean_solve_time = ("Solve Time", "mean"),
            median_solve_time = ("Solve Time", "median"),
            IQR_solve_time = ("Solve Time", lambda x: x.quantile(0.75) - x.quantile(0.25))).reset_index())

    #sort by highest success rate first, then lowest median solve time
    summary = summary.sort_values(["success_percentage", "median_solve_time"], ascending=[False, True])

    #save nu summary csv
    output_path = os.path.join(save_dir, filename)
    summary.to_csv(output_path, index=False)

    return None


def analysis(csv_path: str, group_col: str, filename: str, save_dir: str = "results"):

    df = pd.read_csv(csv_path) #read in experiment data


    #calculate Confidence Interval by group_col
    successful_tests = df[df["Solved (T/F)"] == True]["Solve Time"] #df with only rows with successful tests, then take only time col


    time_mean = np.mean(successful_tests)
    sample_size = len(successful_tests)
    sample_std = np.std(successful_tests) 
    standard_error = sample_std / np.sqrt(sample_size)

    summary = (
        df.groupby(group_col).agg( #group by number of drones to see scalability
            num_tested=("Solved (T/F)", "size"),
            success_percentage = ("Solved (T/F)", lambda x: (x == True).mean() * 100), #get mean solve percentage
            mean_solve_time = ("Solve Time", "mean"),
            median_solve_time = ("Solve Time", "median"),
            IQR_solve_time = ("Solve Time", lambda x: x.quantile(0.75) - x.quantile(0.25)),
            Confience_Interval_95 = ("95% Confidence Interval", scipy.stats.norm.interval(confidence=0.95, loc=time_mean, scale=standard_error))).reset_index())

    #sort by highest success rate first, then lowest median solve time
    summary = summary.sort_values([group_col], ascending=True)

    #save csv
    output_path = os.path.join(save_dir, filename)
    summary.to_csv(output_path, index=False)

    return None

if __name__ == "__main__":

    random.seed(42) #for reproducability in experiments

    #first test nu values with 6 envs only, 20 sec time limit 3*3*6= 26 mins
    #nu_values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.75]
    #nu_envs = ["motion_planning_workspaces/K=5/easy_03.yaml", "motion_planning_workspaces/K=5/easy_05.yaml", "motion_planning_workspaces/K=5/medium_03.yaml", "motion_planning_workspaces/K=5/medium_05.yaml", "motion_planning_workspaces/K=5/hard_03.yaml", "motion_planning_workspaces/K=5/hard_05.yaml", "motion_planning_workspaces/K=3/easy_03.yaml", "motion_planning_workspaces/K=3/easy_05.yaml", "motion_planning_workspaces/K=3/medium_03.yaml", "motion_planning_workspaces/K=3/medium_05.yaml", "motion_planning_workspaces/K=3/hard_03.yaml", "motion_planning_workspaces/K=3/hard_05.yaml",  "motion_planning_workspaces/K=1/easy_03.yaml", "motion_planning_workspaces/K=1/easy_05.yaml", "motion_planning_workspaces/K=1/medium_03.yaml", "motion_planning_workspaces/K=1/medium_05.yaml", "motion_planning_workspaces/K=1/hard_03.yaml", "motion_planning_workspaces/K=1/hard_05.yaml"]
    #test(nu_values, nu_envs, 20, filename="nu_test.csv")
    #summarise_nu("results/nu_test.csv", "nu_summary.csv")
    best_nu = 0.2 #determined from nu_summary.csv in results


    #using best nu value, test 15 envs, 1-5 num drones, 60 sec limit
    #env_path_list = [str(p) for p in list((Path("motion_planning_workspaces").rglob("*.yaml")))] #get list of all test envs
    #print(env_path_list)
    #test([best_nu], env_path_list, time_limit= 20, filename= "eval_w_best_nu.csv")


    #now analyse performance across difficulty settings
    analysis("results/eval_w_best_nu.csv", "Difficulty", "difficulty_analysis.csv")

    #analyse scalability
    analysis("results/eval_w_best_nu.csv", "Number of Drones", "scalability_analysis.csv")
    print("done")