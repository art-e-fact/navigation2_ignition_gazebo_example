import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TransformStamped
import math
import threading
import time
from typing import Tuple


class RobotState:
    """Represents the current state of the robot in simulation"""
    
    def __init__(self):
        self.position = (0.0, 0.0)
        self._lock = threading.Lock()
    
    def update_transform(self, transform_msg: TransformStamped):
        """Update robot state from transform message"""
        with self._lock:
            self.position = (
                transform_msg.transform.translation.x,
                transform_msg.transform.translation.y
            )
    
    def distance_to(self, coordinates: Tuple[float, float]) -> float:
        """Calculate distance from current position to target coordinates"""
        with self._lock:
            dx = self.position[0] - coordinates[0]
            dy = self.position[1] - coordinates[1]
            return math.sqrt(dx**2 + dy**2)


class SimulationStateUtil(Node):
    """Utility class for accessing simulation state in tests"""
    
    def __init__(self):
        super().__init__("simulation_state_util")
        self.get_logger().info("Simulation State Util Started")
        
        # Create callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Robot state
        self.robot = RobotState()
        
        # Data received flag
        self._transform_received = False
        
        # Subscriber
        self._setup_subscriptions()
    
    def _setup_subscriptions(self):
        """Set up ROS2 subscription for ground truth transform"""
        self.transform_sub = self.create_subscription(
            TransformStamped, 
            "/world/worldWrapper/pose", 
            self._transform_callback, 
            10, 
            callback_group=self.callback_group
        )
    
    def _transform_callback(self, msg: TransformStamped):
        """Handle transform messages"""
        self.robot.update_transform(msg)
        self._transform_received = True
    
    def wait_for_initial_state(self, timeout: float = 10.0) -> bool:
        """Wait for initial data from transform subscription"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self._transform_received:
                self.get_logger().info("Initial simulation state received")
                return True
            time.sleep(0.1)
        
        self.get_logger().warn("Timeout waiting for initial simulation state")
        return False
    
    def get(self, entity: str):
        """Get simulation entity by name"""
        if entity == "robot":
            return self.robot
        else:
            raise ValueError(f"Unknown entity: {entity}")
    
    def shutdown_util(self):
        """Clean shutdown of the utility"""
        self.get_logger().info("Shutting down simulation state util")
        self.destroy_node()
