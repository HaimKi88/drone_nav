Drone simulation

- enviroment: 
Ubuntu 22.04.5,
ROS2 humble,
Gazebo 11.10.2.

- dependencies:
sudo apt install ros-humble-geographic-msgs

- launching the simulator:
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch simulation drone_sim.launch.py

