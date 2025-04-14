import io
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

import numpy as np
from PIL import Image

from .vizar_client import VizarClient


ROS_MAP_TOPIC = os.environ.get("ROS_MAP_TOPIC", "/map")
ROS_POSE_TOPIC = os.environ.get("ROS_POSE_TOPIC", "/pose")

VIZAR_SERVER = os.environ.get("VIZAR_SERVER", "http://localhost:5000")
VIZAR_LOCATION = os.environ.get("VIZAR_LOCATION", "ROS Testing")
VIZAR_DEVICE = os.environ.get("VIZAR_DEVICE", "ROS Tester")


class VizarBridge(Node):

    def __init__(self):
        super().__init__('ros2_vizar_bridge')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        if ROS_MAP_TOPIC:
            self.map_subscription = self.create_subscription(OccupancyGrid, ROS_MAP_TOPIC, self.on_map_update, 10)
        if ROS_POSE_TOPIC:
            self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, ROS_POSE_TOPIC, self.on_pose_update, 10)

        self.vizar_client = VizarClient(VIZAR_SERVER, VIZAR_DEVICE, VIZAR_LOCATION)

    def on_map_update(self, msg):
        grid = np.array(msg.data).reshape((msg.info.height, msg.info.width))

        # Flip and rotate for correct orientation of map image
        grid = np.flip(grid, axis=0)
        grid = np.rot90(grid)

        pixels = np.zeros((grid.shape[0], grid.shape[1], 4), dtype=np.uint8)

        # Explored, obstructed -> black, opaque
        pixels[grid >= 75, :] = [0, 0, 0, 255]

        # Explored, clear -> white, opaque
        pixels[(grid >= 0) & (grid < 75), :] = 255

        # Unexplored -> blue-gray, semi-transparent
        pixels[grid < 0, :] = [112, 137, 134, 179]

        im = Image.fromarray(pixels, mode="RGBA")

        buffer = io.BytesIO()
        im.save(buffer, format="PNG")
        buffer.seek(0)

        box = dict(
            height=grid.shape[0] * msg.info.resolution,
            width=grid.shape[1] * msg.info.resolution,
        )
        box['left'] = -msg.info.origin.position.y - box['width']
        box['top'] = msg.info.origin.position.x

        self.vizar_client.update_layer(buffer, box)

    def on_pose_update(self, msg):
        pose = msg.pose.pose

        self.get_logger().info(f"Position {pose.position} and orientation {pose.orientation} in frame {msg.header.frame_id}")
        self.vizar_client.update_pose(pose.position, pose.orientation)


def main(args=None):
    rclpy.init(args=args)

    node = VizarBridge()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
