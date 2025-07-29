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
from gz_interfaces.srv import GetTransform



class GazeboROSNode(ROSNode):
    def __init__(self, recording_path=None):
        super().__init__('gazebo_pose_listener')
        if recording_path is None:
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
        self.srv = self.create_service(GetTransform, 'get_transform', self.handle_get_transform)

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
    
    def distance_to(self, object_name: str, waypoint_coords: Dict) -> Dict[str, float]:
        """Calculate distance between object and waypoint coordinates
        
        Args:
            object_name: Name of the object to check
            waypoint_coords: Dictionary with 'position' and 'orientation' keys
            
        Returns:
            Dictionary with position_error, orientation_error, and success status
        """
        tf = self.transforms.get(object_name)
        
        if tf is None:
            return {'error': f'No transform found for object {object_name}'}
            
        # Calculate position error
        target_pos = waypoint_coords['position']
        object_pos = tf.transform.translation
        
        pos_error = math.sqrt(
            (object_pos.x - target_pos['x']) ** 2 +
            (object_pos.y - target_pos['y']) ** 2 +
            (object_pos.z - target_pos['z']) ** 2
        )
        
        # Calculate orientation error (quaternion distance)
        target_orient = waypoint_coords['orientation']
        object_orient = tf.transform.rotation
        
        dot_product = abs(
            object_orient.x * target_orient['x'] +
            object_orient.y * target_orient['y'] +
            object_orient.z * target_orient['z'] +
            object_orient.w * target_orient['w']
        )
        dot_product = min(1.0, dot_product)  # Clamp to avoid numerical errors
        orient_error = 2.0 * math.acos(dot_product)
        
        result = {
            'position_error': pos_error,
            'orientation_error': orient_error
        }
        
        self.get_logger().info(
            f"Distance from {object_name} to waypoint: pos_err={pos_error:.3f}m, orient_err={orient_error:.3f}rad"
        )
        
        return result
 


def main():
    rclpy.init()
    recording_path = None
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
