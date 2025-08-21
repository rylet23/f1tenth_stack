BACKGROUND

Hello / Merhaba
This repository combines the simulation environment and a perception-based algorithm for the F1TENTH stack. The algorithm uses a model trained in simulation alongside live ZED and LiDAR data, leveraging both a 2D LiDAR scan and the ZED camera for richer perception.
When using only 2D LiDAR, we ran into limitations: the sensor can miss objects below the scan plane (e.g., chair legs) and, being laser-based, often fails to detect glass. With both LiDAR and camera enabled, the car can detect these cases.
My primary goal in the NSF IRES program is to use the F1TENTH car because a smart car uses the same technologies—just at higher quality/scale. For example, a smart car typically uses a 3D LiDAR instead of the 2D unit on the F1 vehicle. Since both return scan data in a dataframe-like format, this approach should be nearly compatible with the smart car (not yet tested, but that’s my expectation).

RUNNING THE SIMULATION

cd ~/Simulation/f1tenth-gym-quickstart/src
python3 NewerSim.py

This uses parameters from NewAttributes.py (same directory) and the maps in the maps folder; NewAttributes.py also generates a pickle model from the simulation.
The maps folder includes several ITU campus variants and generic maps. To add your own, place it there, run python3 convert_to_bw.py, and then select the new map in your simulation.

RUNNING F1TENTH CAR FOR PERCEPTION

You will need 6 terminals open, I'd recommend using "terminate", and for simplicity I'd run this in all terminals:
cd ~/race_car_ws
ws

You must run the following ros2 nodes in different terminals:
ros2 launch vesc_driver vesc_driver_node.launch.py
ros2 run joy joy_node
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

P.S. If there are any errors in the terminals, 95% of the issues can be fixed by just unplugging that cord and plugging it back in :)

Then you will need to run the file to pull the lidar data and push it to a ros node:
python3 PullLidar.py

Lastly, run the file fixed_speed.py
python3 fixed_speed.py

If the car doesn't begin driving, then it may be a dead battery issue. To check this run:
python3 drive.py
If it doesn't drive in a straight line for a second then slow down to a stop, it is a dead/weak battery.
*After several test runs, it seems like the VESC needs a battery with atleast 10V by the way, so maybe try switching the battery with whichever one is connected to the computer itself.

KNOWN ISSUES:
The one issue I ran into was if the car slows down to being within ~.25 meters of an object it doesn't trigger its rewind logic. In order to get it to backup, I'd just put my hand/foot directly in front of the car and it'll trigger a rewind. I also havenn't tested if the vehicle will check behind itself first before a rewind.

FEATURES:
This perception algorithm allows the car to drive fully autonomously, avoiding both moving and stationary objects, choose the best angle and best next "steps" determined by the current steering angle and speed of the vehicle, then drive in that direction. Currently the vehicle is set at a slower speed, but can be adjusted to +/- 5000 rpm.

