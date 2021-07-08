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
git clone https://github.com/open-rmf/free_fleet
git clone https://github.com/eclipse-cyclonedds/cyclonedds
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
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
```

### 2.2 Install Free Fleet Server in ROS2 foxy

Start a new ROS2 workspace, and pull in the necessary repositories,
```bash
mkdir -p ~/server_ws/src
cd ~/server_ws/src
git clone https://github.com/open-rmf/free_fleet
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
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RELEASE
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

The examples of `free_fleet` using multiple clobers is under development. This section will be updated.
