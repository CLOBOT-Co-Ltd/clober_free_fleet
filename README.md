# Clober Free Fleet

## 1. Free Fleet

`free_fleet` is an [open source fleet management system](https://github.com/open-rmf/free_fleet). `free_fleet` can integrate multiple mobile robots which even though doesn't come with its own fleet management system.

You can read the `free_fleet`'s document [here](https://osrf.github.io/ros2multirobotbook/integration_free-fleet.html).

## 2. Installations

At now, [open-rmf](https://github.com/open-rmf) provides the `free_fleet` client based on ROS1 noetic and the `free_fleet` server based on ROS2 foxy.

### 2.1 Install Free Fleet Clinet in ROS1 noetic

Start a new ROS1 workspace, and pull in the necessary repositories,
```bash
mkdir -p ~/client_ws/src
cd ~/client_ws/src
git clone -b clober-dev https://github.com/CLOBOT-Co-Ltd/free_fleet.git
git clone https://github.com/eclipse-cyclonedds/cyclonedds
git clone https://github.com/CLOBOT-Co-Ltd/clober_free_fleet.git
git clone -b noetic-devel https://github.com/CLOBOT-Co-Ltd/clober.git
git clone -b noetic-devel https://github.com/CLOBOT-Co-Ltd/clober_msgs.git
git clone -b noetic-devel https://github.com/CLOBOT-Co-Ltd/clobot_msgs.git
sudo apt-get install ros-noetic-rosgraph-msgs
sudo apt-get install ros-noetic-rosgraph 
```

Install all the dependencies through rosdep,
```bash
cd ~/client_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -yr
```

Source ROS1 and build,
```bash
cd ~/client_ws
source /opt/ros/noetic/setup.bash
colcon build --symlink-install
```

### 2.2 Install Free Fleet Server in ROS2 foxy

Start a new ROS2 workspace, and pull in the necessary repositories,
```bash
mkdir -p ~/server_ws/src
cd ~/server_ws/src
git clone -b clober-dev https://github.com/CLOBOT-Co-Ltd/free_fleet.git
git clone https://github.com/CLOBOT-Co-Ltd/clober_free_fleet_server.git
git clone https://github.com/open-rmf/rmf_internal_msgs
```

Install all the dependencies through rosdep,
```bash
cd ~/server_ws
rosdep install --from-paths src --ignore-src --rosdistro foxy -yr
```

Source ROS2 and build,
```bash
cd ~/server_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
```

## 3. Examples

### 3.1 Barebones Example

This example emulates a running robot and also a running free fleet server,
```bash
source ~/client_ws/install/setup.bash
roslaunch ff_examples_ros1 fake_client.launch
```

The client will then start subscribing to all the necessary topics, and start publishing robot states over DDS to the server. Start the server using
```bash
source ~/server_ws/install/setup.bash
ros2 launch ff_examples_ros2 fake_server.launch.xml
```

To verify that the fake client has been registered, there will be print-outs on the server terminal, otherwise, the ROS2 messages over the `/fleet_states` topic can also be used to verify,
```bash
source ~/server_ws/install/setup.bash
ros2 topic echo /fleet_states
```

### 3.2 Clober Simulation

Launch the clober free fleet client ROS 1(noetic) :
```bash
source ~/client_ws/install/setup.bash
roslaunch clober_ff_client_ros1 clober_world_ff.launch
```

Launch the clober free fleet server int ROS 2(foxy) :
```bash
source ~/server_ws/install/setup.bash
ros2 launch clober_ff_server_ros2 clober_world_ff.xml
```

Verify the topic `/fleet_states` :

```bash
source ~/server_ws/install/setup.bash
ros2 topic echo /fleet_states
```

### 3.3 Multi Clober Simulation

Launch the clober free fleet client ROS 1(noetic) :
```bash
source ~/client_ws/install/setup.bash
roslaunch clober_ff_client_ros1 clober_suntech_ff.launch
```

Launch the clober free fleet server int ROS 2(foxy) :
```bash
source ~/server_ws/install/setup.bash
ros2 launch clober_ff_server_ros2 clober_world_ff.xml
```

[![Clober Free Fleet Simulation](https://img.youtube.com/vi/Hh-bSrm_ZNc/0.jpg)](https://www.youtube.com/watch?v=Hh-bSrm_ZNc "Clober Free Fleet Simulation")


### 3.4 Commands and Requests

There are 3 types of commands/requests that can be sent to the simulated robots through `free_fleet`.

#### 3.4.1 Destination Requests

Destination requests : command the robot to go to the single destination

[![Clober Free Fleet Simulation](https://img.youtube.com/vi/dJ_G-Utje3w/0.jpg)](https://www.youtube.com/watch?v=dJ_G-Utje3w "Clober Free Fleet Simulation")


```bash
ros2 run clober_ff_server_ros2 send_destination_request.py -f FLEET_NAME -r ROBOT_NAME -x 1.725 -y -0.39 --yaw 0.0 -i UNIQUE_TASK_ID
```

example : command the clober which name is clober_0 to go to the destination(x, y, yaw : 0.0, 0.0, 0.0), and the task_id is destination_requests_task.
```bash
ros2 run clober_ff_server_ros2 send_destination_request.py -f clober -r clober_0 -x 0.0 -y 0.0 --yaw 0.0 -i destination_requests_task
```

#### 3.4.2 Path Requests

Path Requests : command the robot to perform the string of destination

```bash
ros2 run clober_ff_server_ros2 send_path_request.py -f FLEET_NAME -r ROBOT_NAME -i UNIQUE_TASK_ID -p '[{"x": 1.725, "y": -0.39, "yaw": 0.0, "level_name": "B1"}, {"x": 1.737, "y": 0.951, "yaw": 1.57, "level_name": "B1"}, {"x": -0.616, "y": 1.852, "yaw": 3.14, "level_name": "B1"}, {"x": -0.626, "y": -1.972, "yaw": 4.71, "level_name": "B1"}]'
```

example : command the clober which name is clober_0 to perform to the string of destination, and the task_id is path_requests_task.

```bash
ros2 run clober_ff_server_ros2 send_path_request.py -f clober -r clober_0 -i path_requests_task -p '[{"x": 0.0, "y": 0.0, "yaw": 0.0, "level_name": "B1"}, {"x": 1.0, "y": 1.0, "yaw": 1.57, "level_name": "B1"}, {"x": -1.0, "y": 0.0, "yaw": 3.14, "level_name": "B1"}, {"x": 0.0, "y": 1.0, "yaw": 4.71, "level_name": "B1"}]'
```

#### 3.4.3 Mode Requests

Mode Requests : command the robot to `pause` or `resume`

[![Clober Free Fleet Simulation](https://img.youtube.com/vi/733UkaXWgEg/0.jpg)](https://www.youtube.com/watch?v=733UkaXWgEg "Clober Free Fleet Simulation")


* pause
```bash
ros2 run clober_ff_server_ros2 send_mode_request.py -f FLEET_NAME -r ROBOT_NAME -m pause -i UNIQUE_TASK_ID
```
example : command the clober which name is clober_0 to pause, and the task_id is mode_requests_pause_task.

```bash
ros2 run clober_ff_server_ros2 send_mode_request.py -f clober -r clober_0 -m pause -i mode_requests_pause_task
```

* resume
```bash
ros2 run ff_examples_ros2 send_mode_request.py -f FLEET_NAME -r ROBOT_NAME -m resume -i UNIQUE_TASK_ID
```

example : command the clober which name is clober_0 to resume, and the task_id is mode_requests_resume_task.

```bash
ros2 run clober_ff_server_ros2 send_mode_request.py -f clober -r clober_0 -m resume -i mode_requests_resume_task
```