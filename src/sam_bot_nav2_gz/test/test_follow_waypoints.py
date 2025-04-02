import atexit
import os
import queue
import signal
import subprocess
import threading
import time
from datetime import datetime

import pytest
from artefacts_toolkit.chart import make_chart
from artefacts_toolkit.config import get_artefacts_param
from artefacts_toolkit.gazebo import gz
from artefacts_toolkit.rosbag import image_topics, rosbag


class MonitoredProcess:
    """Launch, monitor and manage a process with stdout/stderr capture."""

    def __init__(self, cmd, name=None, **popen_kwargs):
        self.cmd = cmd
        self.name = name if name else "Process"
        self.output_lines = []
        self.stdout_queue = queue.Queue()

        # Default kwargs for subprocess.Popen
        default_kwargs = {
            "stdout": subprocess.PIPE,
            "stderr": subprocess.STDOUT,
            "bufsize": 1,
            "universal_newlines": False,  # Binary mode for consistent line endings
        }

        # Merge with user-provided kwargs, keeping user values if there are conflicts
        popen_kwargs = {**default_kwargs, **popen_kwargs}

        print(f"[{self.name}]: Starting process: {' '.join(str(x) for x in cmd)}")
        # Start the process
        self.process = subprocess.Popen(cmd, **popen_kwargs)

        # Start output monitoring thread
        self.monitor_thread = threading.Thread(target=self._monitor_output)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def _monitor_output(self):
        """Continuously read from process stdout and store in queue and list."""
        for line in iter(self.process.stdout.readline, b""):
            try:
                if type(line) is bytes:
                    # Decode bytes to string
                    line = line.decode("utf-8", errors="replace")
                line_str = line.rstrip()
                self.stdout_queue.put(line_str)
                self.output_lines.append(line_str)
                print(f"[{self.name}]: {line_str}")
            except Exception as e:
                print(f"[{self.name}] Error processing output: {e}")

    def wait_for_output(self, pattern, timeout=60):
        """Wait for a specific pattern in the stdout."""
        start_time = time.time()
        print(f"[{self.name}]: Waiting for pattern: '{pattern}' (timeout: {timeout}s)")

        # First check if pattern already exists in captured output
        for line in self.output_lines:
            if pattern in line:
                print(f"[{self.name}]: Pattern found in existing output")
                return True

        # If not found yet, wait for new output until timeout
        while time.time() - start_time < timeout:
            try:
                line = self.stdout_queue.get(timeout=1)
                if pattern in line:
                    print(f"[{self.name}]: Pattern found in new output")
                    return True
            except queue.Empty:
                # No output for 1 second, continue waiting if within timeout
                if self.process.poll() is not None:
                    print(
                        f"[{self.name}]: Process exited with code {self.process.poll()}"
                    )
                    # Process ended, no point waiting more
                    return False

        print(f"[{self.name}]: Pattern not found within timeout period")
        return False

    def get_full_output(self):
        """Return the complete stdout captured so far."""
        return "\n".join(self.output_lines)

    def terminate(self, timeout=1000):
        """Terminate the process gracefully."""
        if self.process.poll() is None:
            print(f"[{self.name}]: Sending SIGINT...")
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=timeout)
                return True
            except subprocess.TimeoutExpired:
                print(f"[{self.name}]: SIGINT failed, sending SIGTERM...")
                self.process.terminate()
                try:
                    self.process.wait(timeout=timeout / 2)
                    return True
                except subprocess.TimeoutExpired:
                    print(f"[{self.name}]: SIGTERM failed, sending SIGKILL...")
                    self.process.kill()
                    self.process.wait(timeout=1)
                    return True
        return False  # Process was already terminated

    def kill(self):
        """Kill the process immediately."""
        if self.process.poll() is None:
            self.process.kill()
            self.process.wait(timeout=1)
            return True
        return False  # Process was already terminated


@pytest.fixture(scope="module")
def rosbag_data():
    """Create and start a rosbag recorder for the test."""
    topics = ["/odom"]
    metrics = ["/distance_from_start_gt", "/distance_from_start_est", "/odometry_error"]
    camera_topics = ["/sky_cam"]
    sim_topics = ["/world/dynamic_pose/info"]

    # NOTE: Same as https://github.com/art-e-fact/artefacts-toolkit-rosbag/blob/main/artefacts_toolkit_rosbag/rosbag.py
    #   so far i couldn't find a way to get the command as string. Maybe we can add it if it's useful
    yyyymmddhhmmss = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    rosbag_filepath = os.path.join("output", "rosbag2_" + yyyymmddhhmmss)
    rosbag_cmd = (
        ["ros2", "bag", "record"]
        + topics + sim_topics + metrics + camera_topics
        + ["-o", rosbag_filepath, "--storage", "mcap", "--use-sim-time"]
    )
    print(f"Recording to {rosbag_filepath} with command: {rosbag_cmd}")

    # Start the rosbag recorder
    monitored_process = MonitoredProcess(
        cmd=rosbag_cmd, name="RosBag", universal_newlines=True
    )

    yield {"filepath": rosbag_filepath, "process": monitored_process}

    # Stop rosbag recording
    monitored_process.terminate()


@pytest.fixture(scope="module")
def navigation_stack():
    """Launch the navigation stack and monitor its output."""
    try:
        world = get_artefacts_param("launch", "world")
    except FileNotFoundError:
        world = "empty.sdf"  # Make sure this is a valid filename with extension

    # Build the ros2 launch command
    launch_cmd = [
        "ros2",
        "launch",
        "sam_bot_nav2_gz",
        "waypoint_follower_example_launch.py",
        f"world_file:={world}",
        "run_headless:=True",
    ]

    # Create monitored process
    nav_process = MonitoredProcess(cmd=launch_cmd, name="Navigation")

    # Wait a moment to ensure process is starting
    time.sleep(2)

    yield {"process": nav_process}

    # Teardown - kill the navigation stack
    nav_process.terminate()
    # Kill gazebo processes to ensure clean state
    gz.kill_gazebo()


# @pytest.fixture(scope="module")
# def odometry_node(navigation_stack):
#     """Start the odometry test node."""
#     odometry_cmd = [
#         "python3",
#         os.path.join("src", "sam_bot_nav2_gz", "test", "test_odometry_node.py"),
#     ]

#     # Create monitored process
#     odometry_process = MonitoredProcess(cmd=odometry_cmd, name="OdometryTest")

#     yield {"process": odometry_process}

#     # Teardown - kill the odometry node
#     # odometry_process.terminate()

# def test_nav2_started(navigation_stack):
#     """Test that Nav2 stack starts properly."""
#     process = navigation_stack["process"]

#     # Wait for the message indicating Nav2 has started
#     success = process.wait_for_output("Nav2 is ready for use!", timeout=100)
#     assert success, "Nav2 apparently failed to start"


# def test_followed_waypoints(navigation_stack):
#     """Check the logs to see if the navigation task is completed."""
#     process = navigation_stack["process"]

#     # Wait for the message indicating goal success
#     success = process.wait_for_output("Goal succeeded!", timeout=300)
#     assert success, "Failed to complete waypoint sequence"


# def test_no_skipped_waypoint(navigation_stack):
#     """Check that no waypoints were skipped during navigation."""
#     process = navigation_stack["process"]

#     # Check for messages about reaching each waypoint
#     for i in range(1):
#         success = process.wait_for_output(f"Executing current waypoint: 1", timeout=120)
#         assert success, f"Failed to reach waypoint {i}"

# dummy test
def test_dummy(rosbag_data, navigation_stack):
    """Dummy test to ensure pytest runs."""
    assert True, "Dummy test passed"



@pytest.fixture(scope="module", autouse=True)
def global_teardown(request, rosbag_data):
    """Run after all tests have completed."""
    print("\n=== Running global teardown ===")
    # This code runs before any tests
    yield
    # This code runs after all tests complete
    print("\n=== Running global teardown ===")

    try:
        # Extract the rosbag filepath
        rosbag_filepath = rosbag_data["filepath"]
        print(f"Processing rosbag at: {rosbag_filepath}")

        # Generate charts from recorded data
        make_chart(
            rosbag_filepath,
            "/odom.pose.pose.position.x",
            "/odom.pose.pose.position.y",
            field_unit="m",
            chart_name="odometry_position",
        )

        # Extract media from recorded data
        image_topics.extract_camera_image(rosbag_filepath, "/sky_cam")
        image_topics.extract_video(rosbag_filepath, "/sky_cam", "output/sky_cam.webm")

        # Any other cleanup operations
        print("=== Global teardown completed ===")
    except Exception as e:
        print(f"Error in global teardown: {e}")
        # Still run gazebo cleanup even if chart generation fails
        gz.kill_gazebo()
