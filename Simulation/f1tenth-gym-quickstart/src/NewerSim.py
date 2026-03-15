import time
import yaml
import gym
import numpy as np
import concurrent.futures
import csv

from starting_point import SimpleDriver
from NewAttributes import NewStuff
from waypoint_driver import PurePursuitDriver

def load_waypoints(csv_file):
    waypoints = []
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            if len(row) >= 2:
                try:
                    x, y = float(row[0]), float(row[1])
                    waypoints.append([x, y])
                except ValueError:
                    continue
    return waypoints

waypoints = load_waypoints('adjusted_smoothed_path.csv')
drivers = [PurePursuitDriver(waypoints)]

RACETRACK = 'my_map2'

log_file = open('drive_data.csv', mode='w', newline='')
log_it = csv.writer(log_file)
log_it.writerow(['step', 'speed', 'steering_angle', 'min_front_dist', 'collision_happen'])

if __name__ == '__main__':
    with open(f'maps/{RACETRACK}.yaml') as map_conf_file:
        map_conf = yaml.load(map_conf_file, Loader=yaml.FullLoader)
    scale = map_conf['resolution'] / map_conf['default_resolution']
    starting_angle = map_conf['starting_angle']

    env = gym.make('f110_gym:f110-v0', map=f"maps/{RACETRACK}", map_ext=".pgm", num_agents=len(drivers))

    poses = np.array([[0.0,0.0, starting_angle]])
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
                    obs['poses_y'][i])
                )

        for future in futures:
            try:
                result = future.result()
                if len(result) == 3:
                    speed, steer, min_front_dist = result
                else:
                    speed, steer = result
                    min_front_dist = -1.0

                collision_imminent = int(min_front_dist >= 0 and min_front_dist < 0.5)
                log_it.writerow([step_counter, speed, steer, min_front_dist, collision_imminent])
                actions.append([steer, speed])
            except Exception as e:
                print(f"❌ Error during future.result(): {e}")
                actions.append([0.0, 0.0]) 
            step_counter += 1

        actions = np.array(actions)
        obs, step_reward, done, info = env.step(actions)
        laptime += step_reward
        env.render(mode='human')

    log_file.close()
    print('Sim elapsed time:', laptime, 'Real elapsed time:', time.time() - start)

