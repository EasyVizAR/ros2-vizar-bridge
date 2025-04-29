# ROS2 VizAR Bridge

Bridge data between ROS2 and VizAR server for AR devices.
This currently supports occupancy grid maps and robot pose.

## Installation

This code is tested with Python 3.12 and ROS2 Jazzy.

Required Python packages:

- numpy
- pillow
- rclpy (from ROS package)
- requests

```bash
sudo apt-get install -y python3-numpy python3-pil python3-requests ros-jazzy-rclpy
```

## Running the Node

We are using several environment variables to configure the node.
The default values are shown below.

```bash
ROS_MAP_TOPIC=/map
ROS_POSE_TOPIC=/pose
VIZAR_SERVER=http://localhost:5000
VIZAR_LOCATION="ROS Testing"
VIZAR_DEVICE="ROS Tester"
```

The default map and pose topics will work for a single robot that publishes
occupancy grids and pose data. For multiple robots, one can run multiple bridge
nodes listening on different topics, e.g. "/robot1/pose" and "/robot2/pose".
If a combined map is available, only one node needs to subscribe to it, e.g.
"/combined/map", and other nodes can set ROS_MAP_TOPIC="" to ignore map
updates.

The VIZAR_SERVER variable needs to be set to a valid server URL. We may use
different server URLs during testing and deployment.

The VIZAR_LOCATION variable can be set to a UUID or location name. The node
will ensure that the location exists on the server. One can use different
locations for testing different maps. If multiple robots should appear on
the same map, they should use the same location name or ID.

The VIZAR_DEVICE variable can be set to a UUID or device name. In some places
this may be referred to as a headset ID. The node will ensure that the device
exists on the server. A different name or ID should be used for each robot.

```bash
python3 -m ros2_vizar_bridge
```
