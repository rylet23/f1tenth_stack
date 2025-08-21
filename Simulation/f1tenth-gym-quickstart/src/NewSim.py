import time
import yaml
import gym
import numpy as np
from argparse import Namespace
import concurrent.futures
import csv

# import your drivers here
from starting_point import SimpleDriver
from NewAttributes import NewStuff
from waypoint_driver import PurePursuitDriver
# choose your drivers here (1-4)
drivers = [NewStuff()]
#drivers = [3]
# choose your racetrack here (TRACK_1, TRACK_2, TRACK_3, OBSTACLES, Newitu3 (ITU Campus), bitmap (hallway outside lab 8006))
RACETRACK = 'bitmap'

log_file = open('drive_data.csv',mode='w', newline='')
log_it = csv.writer(log_file)
log_it.writerow(['step','speed', 'steering_angle', "min_front_dist", 'collision_happen'])

if __name__ == '__main__':
    with open('maps/{}.yaml'.format(RACETRACK)) as map_conf_file:
        map_conf = yaml.load(map_conf_file, Loader=yaml.FullLoader)
    scale = map_conf['resolution'] / map_conf['default_resolution']
    starting_angle = map_conf['starting_angle']
    env = gym.make('f110_gym:f110-v0', map="maps/{}".format(RACETRACK),
            map_ext=".png", num_agents=len(drivers))
    # specify starting positions of each agent
   # poses = np.array([[-1.25*scale + (i * 0.75*scale), 0., starting_angle] for i in range(len(drivers))])
    poses = np.array([[6.25, 14.00, starting_angle]])
    obs, step_reward, done, info = env.reset(poses=poses)
    env.render()

    laptime = 0.0
    start = time.time()
    step_counter = 0
    while not done:
        actions = []
        futures = []
        with concurrent.futures.ThreadPoolExecutor() as executor:
            for i, driver in enumerate(drivers):
                futures.append(executor.submit(
                    driver.process_lidar,
                    obs['scans'][i],
                    obs['poses_x'][i],
                    obs['poses_y'][i]),
                )

        for future in futures:
            try:
                speed, steer, min_front_dist = future.result()
                print(f"Speed: {speed}, Steering: {steer}, Min Front Distance: {min_front_dist}")
                collision_imminent = int(min_front_dist < 0.5)
                log_it.writerow([step_counter, speed, steer, min_front_dist, collision_imminent])
                actions.append([steer, speed])
            except Exception as e:
                print(f"Error during future.result(): {e}")
                actions.append([0.0, 0.0])
            step_counter += 1


        actions = np.array(actions)
        obs, step_reward, done, info = env.step(actions)
        laptime += step_reward
        env.render(mode='human')
    log_file.close()
    # print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time()-start)
