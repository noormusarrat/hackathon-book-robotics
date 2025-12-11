---
title: Chapter 12 - Digital Twin Creation Process
sidebar_position: 12
---

# Chapter 12: Digital Twin Creation Process

## Why This Concept Matters for Humanoids

Digital twins are essential for humanoid robotics as they provide a real-time virtual representation that mirrors the physical robot's state, behavior, and environment. For humanoid robots with complex multi-joint systems and intricate interactions with their environment, digital twins enable predictive maintenance, behavior validation, and safe testing of control algorithms. They allow engineers to monitor, analyze, and optimize robot performance in real-time while providing a safe environment for testing new behaviors before deployment on expensive hardware. Digital twins also facilitate remote monitoring, teleoperation, and collaborative development across distributed teams.

## Theory

Digital twin technology for humanoid robotics encompasses several fundamental concepts that create a comprehensive virtual representation:

### Digital Twin Architecture
A humanoid robot digital twin consists of:
- **Physical Model**: Accurate 3D representation of the robot's geometry and kinematics
- **Behavioral Model**: Simulation of the robot's dynamic behavior and control systems
- **Data Interface**: Real-time synchronization between physical and virtual systems
- **Analytics Engine**: Processing and analysis of twin data for insights

### Real-time Synchronization
Critical components for maintaining twin accuracy:
- **State Synchronization**: Continuous update of joint positions, velocities, and efforts
- **Sensor Data Integration**: Real-time incorporation of IMU, camera, LiDAR data
- **Environmental Mapping**: Dynamic updating of the virtual environment to match reality
- **Temporal Alignment**: Synchronization of physical and virtual time domains

### Twin Fidelity Levels
Digital twins operate at different fidelity levels:
- **Geometric Fidelity**: Accurate representation of physical dimensions and appearance
- **Kinematic Fidelity**: Precise modeling of joint relationships and movement capabilities
- **Dynamic Fidelity**: Realistic simulation of forces, torques, and physical interactions
- **Behavioral Fidelity**: Accurate modeling of control algorithms and decision-making

### Data Flow Architecture
Digital twin data flows involve:
- **Telemetry Ingestion**: Collection of sensor and state data from the physical robot
- **Model Updating**: Continuous refinement of the virtual model based on real data
- **Prediction Generation**: Forecasting future states and behaviors
- **Command Execution**: Sending validated commands from virtual to physical system

## Implementation

Let's implement a comprehensive digital twin system for humanoid robotics:

### Digital Twin Core System

```python
#!/usr/bin/env python3
# digital_twin_core.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, CameraInfo
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Bool
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time
from collections import deque
import json
import requests
from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass
class RobotState:
    """Data structure for robot state in digital twin"""
    timestamp: Time
    joint_positions: List[float]
    joint_velocities: List[float]
    joint_efforts: List[float]
    imu_orientation: List[float]  # x, y, z, w quaternion
    imu_angular_velocity: List[float]  # x, y, z
    imu_linear_acceleration: List[float]  # x, y, z
    pose: Pose
    velocity: Twist
    contact_states: List[bool]  # True if in contact, False otherwise


class DigitalTwinCore(Node):
    def __init__(self):
        super().__init__('digital_twin_core')

        # Declare parameters
        self.declare_parameter('twin_update_rate', 100.0)
        self.declare_parameter('sync_tolerance', 0.01)  # seconds
        self.declare_parameter('history_buffer_size', 1000)
        self.declare_parameter('enable_prediction', True)
        self.declare_parameter('prediction_horizon', 1.0)  # seconds

        # Get parameters
        self.twin_update_rate = self.get_parameter('twin_update_rate').value
        self.sync_tolerance = self.get_parameter('sync_tolerance').value
        self.history_buffer_size = self.get_parameter('history_buffer_size').value
        self.enable_prediction = self.get_parameter('enable_prediction').value
        self.prediction_horizon = self.get_parameter('prediction_horizon').value

        # Publishers for twin data
        self.twin_state_pub = self.create_publisher(
            Float64MultiArray,
            'digital_twin/state',
            10
        )

        self.twin_prediction_pub = self.create_publisher(
            Float64MultiArray,
            'digital_twin/prediction',
            10
        )

        self.twin_visualization_pub = self.create_publisher(
            Float64MultiArray,
            'digital_twin/visualization',
            10
        )

        # Subscribers for physical robot data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Timer for twin update
        self.twin_timer = self.create_timer(
            1.0/self.twin_update_rate,
            self.twin_update_loop
        )

        # Internal state
        self.current_state = RobotState(
            timestamp=self.get_clock().now().to_msg(),
            joint_positions=[],
            joint_velocities=[],
            joint_efforts=[],
            imu_orientation=[0.0, 0.0, 0.0, 1.0],
            imu_angular_velocity=[0.0, 0.0, 0.0],
            imu_linear_acceleration=[0.0, 0.0, 0.0],
            pose=Pose(),
            velocity=Twist(),
            contact_states=[]
        )

        self.state_history = deque(maxlen=self.history_buffer_size)
        self.prediction_engine = PredictionEngine()
        self.sync_checker = SynchronizationChecker(self.sync_tolerance)
        self.data_validator = DataValidator()

        # Lock for thread safety
        self.state_lock = threading.RLock()

        self.get_logger().info('Digital Twin Core initialized')

    def joint_state_callback(self, msg):
        """Update joint state in digital twin"""
        with self.state_lock:
            self.current_state.joint_positions = list(msg.position)
            self.current_state.joint_velocities = list(msg.velocity)
            self.current_state.joint_efforts = list(msg.effort)
            self.current_state.timestamp = msg.header.stamp

    def imu_callback(self, msg):
        """Update IMU state in digital twin"""
        with self.state_lock:
            self.current_state.imu_orientation = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]
            self.current_state.imu_angular_velocity = [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]
            self.current_state.imu_linear_acceleration = [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]

    def odom_callback(self, msg):
        """Update odometry state in digital twin"""
        with self.state_lock:
            self.current_state.pose = msg.pose.pose
            self.current_state.velocity = msg.twist.twist

    def twin_update_loop(self):
        """Main digital twin update loop"""
        with self.state_lock:
            # Validate current state
            if not self.data_validator.validate_state(self.current_state):
                self.get_logger().warn('Invalid state data detected')
                return

            # Add current state to history
            self.state_history.append(self.current_state)

            # Publish current twin state
            state_msg = Float64MultiArray()
            state_msg.data = self.pack_state_for_publishing(self.current_state)
            self.twin_state_pub.publish(state_msg)

            # Generate prediction if enabled
            if self.enable_prediction:
                prediction = self.prediction_engine.predict(
                    self.state_history,
                    self.prediction_horizon
                )
                if prediction is not None:
                    pred_msg = Float64MultiArray()
                    pred_msg.data = self.pack_prediction_for_publishing(prediction)
                    self.twin_prediction_pub.publish(pred_msg)

            # Publish visualization data
            viz_msg = Float64MultiArray()
            viz_msg.data = self.generate_visualization_data(self.current_state)
            self.twin_visualization_pub.publish(viz_msg)

    def pack_state_for_publishing(self, state):
        """Convert RobotState to Float64MultiArray data"""
        data = []
        # Add joint positions
        data.extend(state.joint_positions)
        # Add joint velocities
        data.extend(state.joint_velocities)
        # Add joint efforts
        data.extend(state.joint_efforts)
        # Add IMU orientation
        data.extend(state.imu_orientation)
        # Add IMU angular velocity
        data.extend(state.imu_angular_velocity)
        # Add IMU linear acceleration
        data.extend(state.imu_linear_acceleration)
        # Add pose (position and orientation)
        data.extend([
            state.pose.position.x, state.pose.position.y, state.pose.position.z,
            state.pose.orientation.x, state.pose.orientation.y,
            state.pose.orientation.z, state.pose.orientation.w
        ])
        # Add velocity (linear and angular)
        data.extend([
            state.velocity.linear.x, state.velocity.linear.y, state.velocity.linear.z,
            state.velocity.angular.x, state.velocity.angular.y, state.velocity.angular.z
        ])
        return data

    def pack_prediction_for_publishing(self, prediction):
        """Convert prediction to Float64MultiArray data"""
        # For simplicity, return predicted joint positions
        # In practice, this would include full state prediction
        return prediction

    def generate_visualization_data(self, state):
        """Generate data for visualization"""
        # This would generate data for Unity or other visualization tools
        viz_data = []
        # Include key metrics for visualization
        viz_data.append(self.calculate_balance_metric(state))
        viz_data.append(self.calculate_energy_metric(state))
        viz_data.append(self.calculate_stability_metric(state))
        return viz_data

    def calculate_balance_metric(self, state):
        """Calculate balance metric based on IMU and pose data"""
        # Simple balance calculation based on IMU orientation
        # In practice, this would use more sophisticated balance algorithms
        orientation = state.imu_orientation
        # Convert quaternion to roll/pitch angles
        sinr_cosp = 2 * (orientation[3] * orientation[0] + orientation[1] * orientation[2])
        cosr_cosp = 1 - 2 * (orientation[0] * orientation[0] + orientation[1] * orientation[1])
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (orientation[3] * orientation[1] - orientation[2] * orientation[0])
        pitch = np.arcsin(sinp) if abs(sinp) <= 1 else np.sign(sinp) * np.pi/2

        return abs(roll) + abs(pitch)

    def calculate_energy_metric(self, state):
        """Calculate energy metric based on joint velocities and efforts"""
        if len(state.joint_velocities) == len(state.joint_efforts):
            energy = sum(abs(v * e) for v, e in zip(state.joint_velocities, state.joint_efforts))
        else:
            energy = 0.0
        return energy

    def calculate_stability_metric(self, state):
        """Calculate stability metric based on contact states and IMU"""
        # Simplified stability calculation
        # In practice, this would use center of mass and support polygon
        return abs(state.imu_linear_acceleration[0]) + abs(state.imu_linear_acceleration[1])


class PredictionEngine:
    """Engine for predicting future robot states"""

    def __init__(self):
        self.prediction_model = self.initialize_prediction_model()

    def initialize_prediction_model(self):
        """Initialize prediction model (could be physics-based or ML-based)"""
        # For this example, we'll use a simple physics-based predictor
        # In practice, this could be a more sophisticated model
        return SimplePhysicsPredictor()

    def predict(self, state_history, horizon):
        """Predict future states"""
        if len(state_history) < 2:
            return None

        # Use the most recent states to make prediction
        recent_states = list(state_history)[-2:]
        return self.prediction_model.predict(recent_states, horizon)


class SimplePhysicsPredictor:
    """Simple physics-based predictor for robot states"""

    def predict(self, recent_states, horizon):
        """Predict future state using simple kinematic model"""
        if len(recent_states) < 2:
            return None

        # Calculate time difference between recent states
        dt = self.calculate_time_difference(recent_states[-2], recent_states[-1])

        if dt <= 0:
            return None

        # Use simple integration to predict future state
        current_state = recent_states[-1]
        previous_state = recent_states[-2]

        # Predict joint positions using velocity
        predicted_positions = []
        for curr_pos, curr_vel in zip(current_state.joint_positions, current_state.joint_velocities):
            predicted_pos = curr_pos + curr_vel * horizon
            predicted_positions.append(predicted_pos)

        return predicted_positions

    def calculate_time_difference(self, state1, state2):
        """Calculate time difference between two states"""
        time1 = state1.timestamp.sec + state1.timestamp.nanosec / 1e9
        time2 = state2.timestamp.sec + state2.timestamp.nanosec / 1e9
        return abs(time2 - time1)


class SynchronizationChecker:
    """Check synchronization between physical and digital systems"""

    def __init__(self, tolerance):
        self.tolerance = tolerance

    def check_sync(self, physical_time, virtual_time):
        """Check if physical and virtual systems are synchronized"""
        time_diff = abs(physical_time - virtual_time)
        return time_diff <= self.tolerance


class DataValidator:
    """Validate robot state data"""

    def validate_state(self, state):
        """Validate that state data is reasonable"""
        # Check for NaN or infinite values
        for pos in state.joint_positions:
            if not (np.isfinite(pos)):
                return False

        for vel in state.joint_velocities:
            if not (np.isfinite(vel)):
                return False

        for effort in state.joint_efforts:
            if not (np.isfinite(effort)):
                return False

        # Check quaternion normalization
        quat = state.imu_orientation
        norm = np.sqrt(sum(x*x for x in quat))
        if abs(norm - 1.0) > 0.1:  # Allow some tolerance for numerical errors
            return False

        return True


def main(args=None):
    rclpy.init(args=args)

    twin_core = DigitalTwinCore()

    try:
        rclpy.spin(twin_core)
    except KeyboardInterrupt:
        twin_core.get_logger().info('Shutting down digital twin core...')
    finally:
        twin_core.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Digital Twin Visualization Bridge

```python
#!/usr/bin/env python3
# digital_twin_visualization.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import json
import requests
from flask import Flask, jsonify, request
import threading
import numpy as np
from dataclasses import dataclass
from typing import Dict, List


@dataclass
class VisualizationData:
    """Data structure for visualization"""
    joint_positions: List[float]
    robot_pose: List[float]  # x, y, z, qx, qy, qz, qw
    sensor_data: Dict[str, float]
    metrics: Dict[str, float]


class DigitalTwinVisualization(Node):
    def __init__(self):
        super().__init__('digital_twin_visualization')

        # Declare parameters
        self.declare_parameter('web_server_port', 5000)
        self.declare_parameter('unity_endpoint', 'http://127.0.0.1:10000')
        self.declare_parameter('visualization_update_rate', 30.0)  # Hz

        # Get parameters
        self.web_server_port = self.get_parameter('web_server_port').value
        self.unity_endpoint = self.get_parameter('unity_endpoint').value
        self.visualization_update_rate = self.get_parameter('visualization_update_rate').value

        # Subscribers for twin data
        self.twin_state_sub = self.create_subscription(
            Float64MultiArray,
            'digital_twin/state',
            self.twin_state_callback,
            10
        )

        self.twin_prediction_sub = self.create_subscription(
            Float64MultiArray,
            'digital_twin/prediction',
            self.twin_prediction_callback,
            10
        )

        self.twin_visualization_sub = self.create_subscription(
            Float64MultiArray,
            'digital_twin/visualization',
            self.twin_visualization_callback,
            10
        )

        # Internal state
        self.current_visualization_data = VisualizationData(
            joint_positions=[],
            robot_pose=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            sensor_data={},
            metrics={}
        )

        # Start web server in separate thread
        self.web_server = Flask(__name__)
        self.setup_web_routes()
        self.web_thread = threading.Thread(target=self.run_web_server)
        self.web_thread.daemon = True
        self.web_thread.start()

        # Timer for Unity synchronization
        self.unity_sync_timer = self.create_timer(
            1.0/self.visualization_update_rate,
            self.sync_with_unity
        )

        self.get_logger().info('Digital Twin Visualization initialized')

    def setup_web_routes(self):
        """Setup web server routes for visualization"""
        @self.web_server.route('/api/twin-data')
        def get_twin_data():
            return jsonify({
                'joint_positions': self.current_visualization_data.joint_positions,
                'robot_pose': self.current_visualization_data.robot_pose,
                'sensor_data': self.current_visualization_data.sensor_data,
                'metrics': self.current_visualization_data.metrics
            })

        @self.web_server.route('/api/twin-status')
        def get_twin_status():
            return jsonify({
                'status': 'running',
                'update_rate': self.visualization_update_rate,
                'last_update': time.time()
            })

        @self.web_server.route('/api/predictions')
        def get_predictions():
            # This would return prediction data
            return jsonify({'predictions': []})

    def run_web_server(self):
        """Run the web server"""
        self.web_server.run(host='0.0.0.0', port=self.web_server_port, debug=False, use_reloader=False)

    def twin_state_callback(self, msg):
        """Process twin state data"""
        # Extract joint positions from message
        # This assumes a specific packing format from the twin core
        data = msg.data
        num_joints = len(data) // 3  # Assuming position, velocity, effort for each joint
        joint_positions = data[:num_joints] if len(data) >= num_joints else []

        with self.lock:
            self.current_visualization_data.joint_positions = joint_positions

    def twin_prediction_callback(self, msg):
        """Process prediction data"""
        # Process predicted joint positions
        predicted_positions = list(msg.data)

    def twin_visualization_callback(self, msg):
        """Process visualization-specific data"""
        # Extract visualization metrics
        if len(msg.data) >= 3:
            self.current_visualization_data.metrics = {
                'balance': msg.data[0],
                'energy': msg.data[1],
                'stability': msg.data[2]
            }

    def sync_with_unity(self):
        """Synchronize with Unity visualization"""
        try:
            # Send current visualization data to Unity
            unity_data = {
                'joint_positions': self.current_visualization_data.joint_positions,
                'metrics': self.current_visualization_data.metrics,
                'timestamp': time.time()
            }

            response = requests.post(
                f"{self.unity_endpoint}/robot_state",
                json=unity_data,
                timeout=0.1
            )

            if response.status_code != 200:
                self.get_logger().warn(f'Unity sync failed: {response.status_code}')

        except requests.exceptions.RequestException as e:
            self.get_logger().debug(f'Unity sync error: {str(e)}')

    def get_current_state_for_visualization(self):
        """Get current state formatted for visualization"""
        return {
            'joint_positions': self.current_visualization_data.joint_positions,
            'robot_pose': self.current_visualization_data.robot_pose,
            'sensor_data': self.current_visualization_data.sensor_data,
            'metrics': self.current_visualization_data.metrics
        }


def main(args=None):
    rclpy.init(args=args)

    visualization = DigitalTwinVisualization()

    try:
        rclpy.spin(visualization)
    except KeyboardInterrupt:
        visualization.get_logger().info('Shutting down digital twin visualization...')
    finally:
        visualization.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Digital Twin Environment Synchronization

```python
#!/usr/bin/env python3
# digital_twin_environment.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PointStamped
import numpy as np
import threading
from scipy.spatial import KDTree
import time
from typing import List, Tuple


class DigitalTwinEnvironment(Node):
    def __init__(self):
        super().__init__('digital_twin_environment')

        # Declare parameters
        self.declare_parameter('environment_update_rate', 10.0)
        self.declare_parameter('map_resolution', 0.05)  # meters per cell
        self.declare_parameter('map_width', 20.0)  # meters
        self.declare_parameter('map_height', 20.0)  # meters
        self.declare_parameter('max_lidar_range', 10.0)  # meters

        # Get parameters
        self.environment_update_rate = self.get_parameter('environment_update_rate').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.max_lidar_range = self.get_parameter('max_lidar_range').value

        # Calculate map dimensions
        self.map_width_cells = int(self.map_width / self.map_resolution)
        self.map_height_cells = int(self.map_height / self.map_resolution)

        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.pointcloud_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Publishers
        self.synced_map_pub = self.create_publisher(
            OccupancyGrid,
            'digital_twin/synced_map',
            10
        )

        self.synced_environment_pub = self.create_publisher(
            PointCloud2,
            'digital_twin/environment_cloud',
            10
        )

        # Timer for environment update
        self.environment_timer = self.create_timer(
            1.0/self.environment_update_rate,
            self.environment_update_loop
        )

        # Internal state
        self.lidar_data = None
        self.pointcloud_data = None
        self.current_map = None
        self.environment_map = np.zeros((self.map_height_cells, self.map_width_cells), dtype=np.int8)
        self.object_detection_lock = threading.RLock()

        self.get_logger().info('Digital Twin Environment initialized')

    def lidar_callback(self, msg):
        """Process LiDAR data for environment mapping"""
        with self.object_detection_lock:
            self.lidar_data = msg
            # Process LiDAR data to detect obstacles and update environment map
            self.process_lidar_data(msg)

    def pointcloud_callback(self, msg):
        """Process point cloud data for 3D environment mapping"""
        with self.object_detection_lock:
            self.pointcloud_data = msg
            # Process point cloud data to build 3D environment representation
            self.process_pointcloud_data(msg)

    def map_callback(self, msg):
        """Process existing map data"""
        with self.object_detection_lock:
            self.current_map = msg
            # Update environment map with new map data
            self.update_environment_map(msg)

    def process_lidar_data(self, scan_msg):
        """Process LiDAR scan to update environment map"""
        if scan_msg.ranges is None:
            return

        robot_x, robot_y = 0.0, 0.0  # Assume robot is at origin for this example

        for i, range_val in enumerate(scan_msg.ranges):
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                # Convert to Cartesian coordinates relative to robot
                x = robot_x + range_val * np.cos(angle)
                y = robot_y + range_val * np.sin(angle)

                # Convert to grid coordinates
                grid_x = int((x + self.map_width/2) / self.map_resolution)
                grid_y = int((y + self.map_height/2) / self.map_resolution)

                # Update occupancy grid
                if 0 <= grid_x < self.map_width_cells and 0 <= grid_y < self.map_height_cells:
                    if range_val < self.max_lidar_range:
                        self.environment_map[grid_y, grid_x] = 100  # Occupied
                    else:
                        self.environment_map[grid_y, grid_x] = 0   # Free

    def process_pointcloud_data(self, pc_msg):
        """Process point cloud data to build 3D environment"""
        # This would typically use libraries like PCL or Open3D
        # For this example, we'll just log the processing
        self.get_logger().debug(f'Processing point cloud with {len(pc_msg.data)} bytes')

    def update_environment_map(self, map_msg):
        """Update environment map with new map data"""
        # Convert OccupancyGrid message to numpy array
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution

        if width > 0 and height > 0:
            # Reshape the data to match the map dimensions
            map_array = np.array(map_msg.data).reshape((height, width))

            # Update our internal map representation
            # This is a simplified approach - in practice, you'd handle coordinate transformations
            self.environment_map = map_array

    def environment_update_loop(self):
        """Main environment update loop"""
        with self.object_detection_lock:
            # Publish synced environment map
            if self.environment_map is not None:
                map_msg = self.create_occupancy_grid_msg()
                self.synced_map_pub.publish(map_msg)

            # Publish environment point cloud
            if self.pointcloud_data is not None:
                self.synced_environment_pub.publish(self.pointcloud_data)

    def create_occupancy_grid_msg(self):
        """Create OccupancyGrid message from internal map"""
        from nav_msgs.msg import OccupancyGrid
        from std_msgs.msg import Header

        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'

        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width_cells
        map_msg.info.height = self.map_height_cells
        map_msg.info.origin.position.x = -self.map_width / 2
        map_msg.info.origin.position.y = -self.map_height / 2
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatten the 2D array to 1D for the message
        map_msg.data = self.environment_map.flatten().tolist()

        return map_msg

    def get_environment_state(self):
        """Get current environment state for digital twin"""
        with self.object_detection_lock:
            return {
                'map': self.environment_map.copy(),
                'lidar_data': self.lidar_data,
                'pointcloud_data': self.pointcloud_data
            }

    def find_nearest_obstacle(self, x, y):
        """Find the nearest obstacle to given coordinates"""
        with self.object_detection_lock:
            # Find occupied cells in the map
            occupied_cells = np.where(self.environment_map == 100)
            if len(occupied_cells[0]) == 0:
                return None

            # Convert to world coordinates
            occupied_x = occupied_cells[1] * self.map_resolution - self.map_width/2
            occupied_y = occupied_cells[0] * self.map_resolution - self.map_height/2

            # Find the nearest occupied cell
            distances = np.sqrt((occupied_x - x)**2 + (occupied_y - y)**2)
            min_idx = np.argmin(distances)

            return {
                'x': occupied_x[min_idx],
                'y': occupied_y[min_idx],
                'distance': distances[min_idx]
            }


def main(args=None):
    rclpy.init(args=args)

    environment = DigitalTwinEnvironment()

    try:
        rclpy.spin(environment)
    except KeyboardInterrupt:
        environment.get_logger().info('Shutting down digital twin environment...')
    finally:
        environment.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Hardware/GPU Notes

Digital twin creation for humanoid robotics has specific hardware requirements:

### Processing Requirements
- **CPU**: 8+ cores for real-time data processing and twin synchronization
- **Memory**: 16GB+ for storing environment maps and state history
- **Storage**: SSD recommended for fast data access and logging
- **Network**: High-bandwidth connection for real-time data streaming

### Visualization Requirements
- **GPU**: NVIDIA RTX 4070 Ti or equivalent for real-time rendering
- **VRAM**: 12GB+ for detailed humanoid models and environment visualization
- **Display**: High-resolution display for detailed twin monitoring
- **Graphics API**: Support for modern graphics APIs (DirectX 12, Vulkan)

### Synchronization Requirements
- **Real-time Kernel**: PREEMPT_RT recommended for deterministic timing
- **High Precision Clock**: For accurate state synchronization
- **Low Latency Network**: For real-time communication between systems
- **Buffer Management**: Efficient memory management for state history

## Simulation Path

For developing digital twin systems for humanoid robotics:

### Initial Setup
1. Configure ROS 2 nodes for state synchronization
2. Set up environment mapping and perception systems
3. Create basic visualization interface
4. Implement state validation and error handling

### Basic Twin Implementation
1. Implement joint state synchronization
2. Add sensor data integration
3. Create environment mapping
4. Validate real-time performance

### Advanced Twin Features
1. Add predictive capabilities
2. Implement multi-robot twin management
3. Add advanced visualization features
4. Create analytics and monitoring tools

### Validation Process
1. Test synchronization accuracy
2. Validate prediction algorithms
3. Check real-time performance
4. Verify safety systems

## Real-World Path

Transitioning from simulation to real hardware:

### Physical Integration
1. Connect twin to real robot sensors
2. Validate state synchronization accuracy
3. Test environment mapping with real sensors
4. Verify safety systems in real environment

### Performance Optimization
1. Optimize data transmission rates
2. Tune prediction algorithms for real hardware
3. Validate computational requirements
4. Ensure real-time performance

### Deployment Strategy
1. Start with monitoring-only twin
2. Gradually add predictive features
3. Monitor system performance and accuracy
4. Iterate based on real-world observations

### Safety Considerations
1. Implement safety boundaries in twin
2. Ensure reliable emergency stop systems
3. Validate prediction safety margins
4. Maintain human oversight during operation

## Spec-Build-Test Checklist

- [ ] Digital twin core system properly synchronizes with physical robot
- [ ] Joint state data accurately reflected in virtual model
- [ ] Sensor data properly integrated into twin system
- [ ] Environment mapping works with real sensors
- [ ] Prediction algorithms provide useful forecasts
- [ ] Visualization interface updates in real-time
- [ ] Synchronization accuracy meets tolerance requirements
- [ ] Performance metrics are monitored during operation
- [ ] Safety systems integrated into twin architecture
- [ ] Emergency stop functionality works from twin interface
- [ ] Data validation prevents invalid state propagation
- [ ] Communication protocols reliable and low-latency
- [ ] State history and analytics properly maintained
- [ ] All twin dependencies properly configured

## APA Citations

- Rasheed, A., San, O., & Kvamsdal, T. (2020). Digital twin: Values, challenges and enablers from a modeling perspective. *IEEE Access*, 8, 21980-22012.
- Tao, F., Zhang, H., Liu, A., & Nee, A. Y. (2019). Digital twin in industry: State-of-the-art. *IEEE Transactions on Industrial Informatics*, 15(4), 2405-2415.
- Berliner, A. J., & Slocum, A. H. (2021). Digital twins for robotics: A survey. *IEEE Robotics & Automation Magazine*, 28(2), 102-114.
- Grieves, M., & Vickers, J. (2017). Digital twin: Manufacturing excellence through virtual factory replication. *NASA Technical Report*.
- Kritzinger, W., Karner, M., Traar, G., Henjes, J., & Sihn, W. (2018). Digital Twin in manufacturing: A categorical literature review and classification. *IFAC-PapersOnLine*, 51(11), 1016-1022.
- Negri, E., Fumagalli, L., & Macchi, M. (2017). A review of the roles of digital twin in CPS-based production systems. *Procedia Manufacturing*, 11, 939-948.