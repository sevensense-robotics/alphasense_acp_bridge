# Installing packages from APT

## Setting up APT repository

To install the Alphasense Communication Protocol (ACP) bridge on Ubuntu 20.04 + ROS-noetic and Ubuntu 18.04 + ROS-melodic systems with amd64 or arm64 architecture we recommend installation over APT.

```sh
# Install curl for the next step
sudo apt install curl

# Add the Sevensense PGP key to make this machine trust Sevensense's packages.
curl -Ls https://deb.7sr.ch/pubkey.gpg | sudo gpg --dearmor -o /usr/share/keyrings/deb-7sr-ch-keyring.gpg

# Add the Sevensense APT repository to the list of known sources.
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/deb-7sr-ch-keyring.gpg] https://deb.7sr.ch/alphasense $(lsb_release -cs) main" \
          | sudo tee /etc/apt/sources.list.d/sevensense.list
```

This should result in a file `/etc/apt/sources.list.d/sevensense.list` with the following content.

```
deb [arch=amd64 signed-by=/usr/share/keyrings/deb-7sr-ch-keyring.gpg] https://devel.deb.7sr.ch/alphasense focal main
```

## Install the bridge

Generally the package and its dependencies can be installed with on ROS-noetic

```sh
sudo apt install ros-noetic-sev-acp-external-bridge
```
or on ROS-melodic
```sh
sudo apt install ros-melodic-sev-acp-external-bridge
```

# Using the bridge

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

## Sending odometry data to an Alphasense Embedded

To send odometry data to the Alphasense Embedded, start the bridge with the `--odometry-topic` and specify the topic where `geometry_msgs/TwistStamped` messages get published and use the `--alphasense-ip` argument to specify the IP of the Alphasense Embedded:

```sh
rosrun sev_acp_external_bridge ap-udp-bridge --alphasense-ip 192.168.76.100 --odometry-topic /robot/velocity
```

Alphasense Embedded listens on port `7777` and by default the bridge sends to port `7777`. The behaviour of the bridge can be changed by using the `--alphasense-port` argument.

## Receiving pose information from Alphasense Embedded

Use the web interface of the Alphasense Embedded to specify the IP address where it should send position information to. E.g. setting on the webinterface `192.168.76.1:7777` will send position information to that address. The command line argument `--pose-topic PREFIX` enables the listening capabilities of the bridge and publishes two ROS topics. Under `PREFIX/ros_pose` the positions are published as `geometry_msgs/PoseStamped` and under `PREFIX/positioning_update` as `atlas_msgs/PositioningUpdate`. The bridge listens by default on port `7777`, which can be changed with the `--pose-port` argument. For example

```sh
rosrun sev_acp_external_bridge ap-udp-bridge --pose-topic /alphasense --pose-port 7777
```

publishes the topics `/alphasense/ros_pose` and `/alphasense/positioning_update`.

## Bidirectional mode

The bridge can simultaneously receive position data from Alphasense Embedded and send odometry to it.

```sh
rosrun sev_acp_external_bridge ap-udp-bridge --pose-topic /alphasense --pose-port 7777 --alphasense-ip 192.168.76.100 --odometry-topic /robot/velocity
```

## Running multiple bridge instances

The bridge runs by default as a ROS node with the name `ap_udp_bridge`. This name can be changed with the command line argument `__name:=DIFFERENT_NAME` (see the [ROS documentation](https://wiki.ros.org/Nodes#Remapping_Arguments.Special_keys) for this special parameter). This allows running more than once instance of the bridge on one roscore.

```sh
rosrun sev_acp_external_bridge ap-udp-bridge --alphasense-ip 192.168.76.100 --odometry-topic /robot/velocity __name:=odometry_bridge &
rosrun sev_acp_external_bridge ap-udp-bridge --pose-topic /alphasense --pose-port 7777 __name:=pose_bridge
```
