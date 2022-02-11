# Alphasense Communication Protocol (ACP) to ROS bridge 

This repository contains the source code of the ACP-ROS bridge for Alphasense Position, a product by [Sevensense Robotics](https://www.sevensense.ai/).

Alphasense Position is a multi-camera, industrial grade, Visual-SLAM-in-a-box product for mobile robots. 
The system is an Edge AI solution that provides full 3D positioning for any kind of ground vehicles in every environment; even in fast-changing and dynamic spaces.
For more details, visit the [product page](https://www.sevensense.ai/product/alphasense-position).

## The purpose of the bridge

The bridge is used to convert the Alphasense Communication Protocol UDP messages sent over the network to ROS messages on the platform computer. 
In this way, the ROS system of your platform computer can function indepedently of Alphasense Position.

As input, the bridge takes a `TwistStamped` message of ROS, and converts it to UDP messages. 
Similarly, it converts the output of Alphasense Position UDP message to a standard ROS `PoseStamped` message and a custom `PositioningUpdate`. 
See the documentation of Alphansense Position supplied with the product.


# Installation

## Installing packages from APT (Ubuntu systems)

### Setting up APT repository

To install the Alphasense Communication Protocol (ACP) bridge on Ubuntu 20.04 + ROS-noetic and Ubuntu 18.04 + ROS-melodic systems with amd64 or arm64 architecture we recommend installation over APT.

The APT repository details are listed in the Alphasense Position manual that is provided with the unit.

### Install the bridge

Generally the package and its dependencies can be installed with on ROS-noetic

```sh
sudo apt update
sudo apt install ros-noetic-sev-acp-external-bridge
```
or on ROS-melodic
```sh
sudo apt update
sudo apt install ros-melodic-sev-acp-external-bridge
```

## Building from source

All three packages (`acp`, `acp_external_bridge`, `acp_ros_conversions`, `acp_udp`) are configured as catkin packages. 
You can place them in your catkin workspace and build them.

For more information on catkin usage, visit the [catkin Tutorials page](http://wiki.ros.org/catkin/Tutorials).

As of now, the following dependencies still need to be installed from the APT repository introduced above:
- `sevencpp_build`
- `state_machine_msgs`
- `atlas_msgs`
- `minkindr_conversions`
- `cuckoo_time_translator`

These dependencies have been tested in combination with `ROS Noetic` and can be installed with the following command.
```
sudo apt update && sudo apt install sevencpp-build ros-<distro>-state-machine-msgs ros-<distro>-atlas-msgs ros-<distro>-minkindr-conversions ros-<distro>-cuckoo-time-translator
```

# Using the bridge

**Starting with version v8.0.0 of the bridge** all functions are enabled by default and command line arguments have changed with respect to v7.0.1.

The bridge can be started with through `rosrun` and its parameters are shown in the help message.

```sh
rosrun sev_acp_external_bridge ap-udp-bridge --help
```

Both, sending UDP messages and receiving UDP messages, are disabled by default.

The bridge is a ros node and thus assumes you have a running roscore and [environment variables](https://wiki.ros.org/ROS/EnvironmentVariables) set up correspondingly. This is usually done through the ROS-setup `source /opt/ros/current/setup.bash`.

```sh
# shell 1
source /opt/ros/current/setup.bash
roscore
```

```sh
# shell 2
source /opt/ros/current/setup.bash
rosrun sev_acp_external_bridge ap-udp-bridge ...
```

## Sending odometry data to an Alphasense Position

To send odometry data to the Alphasense Position, the bridge subscribes to the topic `/robot/velocity`. This can be changed with the `--odometry-topic` command line argument. The message type on that topic must be `geometry_msgs/TwistStamped`. Specify the IP of the Alphasense Position with the `--alphasense-ip` argument.

```sh
rosrun sev_acp_external_bridge ap-udp-bridge --alphasense-ip 192.168.76.100 --odometry-topic /robot/velocity
```

Alphasense Position listens on port `7777` and by default the bridge sends to port `7777`. The behaviour of the bridge can be changed by using the `--alphasense-port` argument.

## Receiving pose information from Alphasense Position

Use the web interface of the Alphasense Position to specify the IP address where it should send position information to. E.g. setting on the webinterface `192.168.76.1:7777` will send position information to that address.
The bridge publishes data from the Alphasense Position under the following topics, the command line arguments can be used to specify a different topic:

```{list-table}
* - **Message type**
  - `PositioningUpdate`
* - **Topic name**
  - `/alphasense_position/T_G_O_propagated_update`
* - **CLI argument**
  - `--positioning-update-topic`
```
```{list-table}
* - **Message type**
  - `geometry_msgs/PoseStamped`
* - **Topic name**
  - `/alphasense_position/T_G_O_propagated`
* - **CLI argument**
  - `--ros-pose-topic`
```
```{list-table}
* - **Message type**
  - `state_machine_msgs/Status`
* - **Topic name**
  - `/alphasense_position/notifications`
* - **CLI argument**
  - `--notification-topic`
```
```{list-table}
* - **Message type**
  - `state_machine_msgs/State`
* - **Topic name**
  - `/alphasense_position/operation_state`
* - **CLI argument**
  - `--operation-state-topic`
```

The bridge listens by default on port `7777`, which can be changed with the `--local-port` argument.

### Example launch commands

Putting the above together, in the most basic setup only the Alphasense Position IP address needs to be provided to launch the bridge.

```sh
rosrun sev_acp_external_bridge ap-udp-bridge --alphasense-ip 192.168.76.100
```

A more complex command, overriding all default settings can be

```sh
rosrun sev_acp_external_bridge ap-udp-bridge --alphasense-ip 192.168.76.100 --alphasense-port 88 --local-port 7778 --odometry-topic /odometry --positioning-update-topic /alphasense/positioning --ros-pose-topic /ros/pose --notification-topic /notifications --operation-state-topic /AP/operations
```

# License

The code is published under the BSD-3-Clause License, see [the license file](LICENSE).

Copyright (c) 2022, Sevensense Robotics AG
