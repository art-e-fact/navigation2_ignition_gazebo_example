import subprocess
import re
import math
import time
import threading

from typing import Dict, Optional
from xml.etree import ElementTree as ET
import sys
sys.path = sys.path + ['/usr/lib/python3/dist-packages']
from mcap_protobuf.writer import Writer
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


class Pose:
    def __init__(self, name: str, position: Dict[str, float], orientation: Dict[str, float]):
        self.name = name
        self.position = position
        self.orientation = orientation

    def distance_to(self, other: "Pose") -> float:
        dx = self.position.get("x", 0) - other.position.get("x", 0)
        dy = self.position.get("y", 0) - other.position.get("y", 0)
        dz = self.position.get("z", 0) - other.position.get("z", 0)
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def is_near(self, other: "Pose", threshold: float = 0.5) -> bool:
        return self.distance_to(other) < threshold



class Gazebo:
    def __init__(self, recording_path: str = None):
        self.poses: Dict[str, Pose] = {}
        self.node = Node()
        world_name = "collision_test"
        self.topic_dynamicposes = f"/world/{world_name}/dynamic_pose/info"
        self._recording = False
        self._mcap_writer = None
        self._mcap_file = None
        if recording_path is not None:
            self._mcap_file = open(recording_path, "wb")
            self._mcap_writer = Writer(self._mcap_file)
            self._recording = True
        self.node.subscribe(Pose_V, self.topic_dynamicposes, self.posev_cb)
 
    def update_pose(self, name: str, position: Dict[str, float], orientation: Dict[str, float]):
        self.poses[name] = Pose(name, position, orientation)

    def get(self, name: str) -> Optional[Pose]:
        return self.poses.get(name)

    def posev_cb(self, msg: Pose_V):
        if self._recording and self._mcap_writer:
            now = int(time.time() * 1e9)
            self._mcap_writer.write_message(
                topic=self.topic_dynamicposes,
                message=msg,
                log_time=now,
                publish_time=now, # do we compute from msg.header.stamp instead?, or even a sim time topic?
            )
            print("wrote 1 message")
        for pose_msg in msg.pose:
            name = pose_msg.name
            position = {
                "x": getattr(pose_msg.position, "x", 0.0),
                "y": getattr(pose_msg.position, "y", 0.0),
                "z": getattr(pose_msg.position, "z", 0.0),
            }
            orientation = {
                "x": getattr(pose_msg.orientation, "x", 0.0),
                "y": getattr(pose_msg.orientation, "y", 0.0),
                "z": getattr(pose_msg.orientation, "z", 0.0),
                "w": getattr(pose_msg.orientation, "w", 1.0),
            }
            self.update_pose(name, position, orientation)

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join()

        if self._recording:
            if self._mcap_file:
                self._mcap_writer.finish()
                self._mcap_file.close()
            self._recording = False

    def _spin_loop(self):
        while self._running:
            time.sleep(0.001)

def main():
    if len(sys.argv) != 2:
        print("Usage: python recorder.py <output.mcap>")
        sys.exit(1)

    output_path = sys.argv[1]
    gz = Gazebo(recording_path=output_path)
    gz.start()
    print(f"Recording simulation to {output_path}. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping recording...")
        gz.stop()

if __name__ == "__main__":
    main()
