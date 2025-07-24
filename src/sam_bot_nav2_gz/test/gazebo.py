#!/usr/bin/env python3
import subprocess
import re
import math
import time
import threading

from typing import Dict, Optional
from xml.etree import ElementTree as ET
import sys
#sys.path = sys.path + ['/usr/lib/python3/dist-packages']
from mcap_protobuf.writer import Writer
#from gz.msgs10.pose_v_pb2 import Pose_V
#from gz.transport13 import Node

import rclpy
from rclpy.node import Node as ROSNode
from geometry_msgs.msg import TransformStamped
import math
import time
from typing import Dict, Optional
from gz_interfaces.srv import GetTransform  # Define this service as shown below



class GazeboROSNode(ROSNode):
    def __init__(self):
        super().__init__('gazebo_pose_listener')
        recording_path = "/tmp/recording.mcap"
        self.transforms = {}
        world_name = "worldWrapper"
        self.subscription = self.create_subscription(
            TransformStamped,
            f'/world/{world_name}/pose',
            self.transform_cb,
            10
        )
        self._recording = False
        self._mcap_writer = None
        self._mcap_file = None
        if recording_path is not None:
            self._mcap_file = open(recording_path, "wb")
            self._mcap_writer = Writer(self._mcap_file)
            self._recording = True
        self.srv = self.create_service(GetPose, 'get_pose', self.handle_get_transform)

    def update_pose(self, name: str, position: Dict[str, float], orientation: Dict[str, float]):
        self.poses[name] = Pose(name, position, orientation)

    def get(self, name: str) -> Optional[TransformStamped]:
        return self.poses.get(name)

    def transform_cb(self, msg: TransformStamped):
        name = msg.child_frame_id
        self.transforms[name] = msg
        self.get_logger().info(f"Received transform for {name}: {msg.transform}")
        if self._recording and self._mcap_writer:
            now = int(time.time() * 1e9)
            self._mcap_writer.write_message(
                topic='/world/worldWrapper/pose',
                message=msg,
                log_time=now,
                publish_time=now,
            )

    def handle_get_transform(self, request, response):
        tf = self.transforms.get(request.name)
        if tf is not None:
            response.found = True
            response.transform = tf
        else:
            response.found = False
        return response
 


def main():
    rclpy.init()
    if len(sys.argv) > 1:
        recording_path = sys.argv[1]
    node = GazeboROSNode(recording_path=recording_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._mcap_file:
            node._mcap_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
if __name__ == "__main__":
    main()
