from motion_planning import rrt_planner_mab, initialise
import random




#want 15 problems (5 easy, 5 medium, 5 hard according to epsilon, alpha, beta criteria (calculate the values and provide them))
#bounds are fixed
#for each problem, provide different sets of initial and goal states such that I can test the same problem with K in [1,5] drones

#for each problem, # drones combo record the solving time (limit 1 min) or None if no solution found

#average across each difficulty level to get mean and 95% confidence interval

def test(nu_values: list[float], num_drones: list[int], env_path_list: list[str]):

    for env in env_path_list:
        for K in num_drones:
            for nu in nu_values:
            
                sim = initialise(n_drones=K, env_file=env)
                solution_path, solve_time = rrt_planner_mab(sim, nu, 60, env_file=env)


    #TODO make and save csv file with K on one axis, env (with env path as col name) o nthe other axis. the values in each cell is None (if no solution is found) or the time taken to find a solution (if it took less than 60 seconds)

    return None


if __name__ == "__main__":

    random.seed(42) #for reproducability in experiments
    easy_env_path_list = []
    med_env_path_list = []
    hard_env_path_list = []
    nu_values = [0.05, 0.1, 0.25, 0.5, 0.75]
    num_drones = [1,2,3,4,5]

    #first test nu values with 3 envs only, 30 sec time limit 3*3*6= 26 mins
    test(nu_values, [1,3,5], env_path_list)

    #using best nu value, test 15 envs, 1-5 num drones, 60 sec limit
    #test([best_nu], num_drones, env_path_list)