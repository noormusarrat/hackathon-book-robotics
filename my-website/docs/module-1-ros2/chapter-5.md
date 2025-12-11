---
sidebar_position: 5
---

# Chapter 5: ROS 2 for Humanoid Robots

## Why This Concept Matters for Humanoids

ROS 2 provides the essential infrastructure for developing complex humanoid robots, but humanoid-specific challenges require specialized approaches. Unlike simpler robots, humanoids must handle complex kinematics, balance control, coordinated multi-limb motion, and human interaction. This chapter explores how ROS 2's features can be leveraged to address these unique challenges while maintaining the modularity and flexibility that make ROS 2 powerful.

## Theory

Humanoid robots present several unique challenges that require specialized ROS 2 approaches:

**Complex Kinematics**: Humanoid robots typically have 20+ degrees of freedom with complex kinematic chains. This requires sophisticated forward and inverse kinematics solutions that can run in real-time.

**Balance and Locomotion**: Bipedal locomotion requires continuous balance control, making real-time performance and low-latency communication critical.

**Multi-Modal Interaction**: Humanoids must integrate multiple sensory modalities (vision, touch, audio) with complex motor behaviors.

**Safety Requirements**: The human-like form factor creates unique safety challenges, requiring comprehensive safety systems.

**Real-time Constraints**: Many humanoid behaviors require hard real-time performance for stability and safety.

ROS 2 addresses these challenges through:
- **Real-time capabilities**: Support for real-time systems with deterministic behavior
- **Distributed architecture**: Enables modular development of complex systems
- **Communication QoS**: Quality of Service profiles for different message types
- **Lifecycle management**: Proper initialization and cleanup of complex systems
- **Security features**: Authentication and encryption for safe operation

## Implementation

Let's implement a humanoid-specific ROS 2 system that addresses these challenges:

```python
# my_robot_humanoid/my_robot_humanoid/humanoid_manager.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String, Bool, Float64MultiArray
from builtin_interfaces.msg import Time
import numpy as np
import threading
import time
from collections import deque


class HumanoidManager(Node):
    def __init__(self):
        super().__init__('humanoid_manager')

        # Define QoS profiles for different types of data
        self.sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.control_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribers for sensor data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            self.sensor_qos)

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            self.sensor_qos)

        self.tf_sub = self.create_subscription(
            # In a real system, this would be tf2_msgs/TFMessage
            # For this example, using a simple pose message
            Pose,
            '/robot_pose',
            self.pose_callback,
            self.sensor_qos)

        # Publishers for control and status
        self.status_pub = self.create_publisher(
            String,
            '/humanoid_status',
            self.status_qos)

        self.safety_pub = self.create_publisher(
            Bool,
            '/safety_status',
            self.status_qos)

        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/humanoid_commands',
            self.control_qos)

        # Services for humanoid-specific operations
        self.stand_srv = self.create_service(
            String,
            'request_stand',
            self.request_stand_callback)

        self.walk_srv = self.create_service(
            String,
            'request_walk',
            self.request_walk_callback)

        self.sit_srv = self.create_service(
            String,
            'request_sit',
            self.request_sit_callback)

        # Internal state
        self.joint_states = {}
        self.imu_data = {}
        self.robot_pose = Pose()
        self.current_behavior = 'idle'
        self.safety_enabled = True
        self.emergency_stop = False

        # Joint state history for velocity estimation
        self.joint_history = {}
        self.history_length = 5

        # Safety thresholds
        self.safety_thresholds = {
            'imu_angular_velocity': 10.0,  # rad/s
            'imu_linear_acceleration': 50.0,  # m/s^2
            'joint_velocity': 10.0,  # rad/s
            'joint_effort': 100.0  # Nm (example)
        }

        # Timer for main control loop
        self.main_loop_timer = self.create_timer(0.01, self.main_control_loop)  # 100 Hz

        # Timer for safety monitoring
        self.safety_timer = self.create_timer(0.005, self.safety_monitor)  # 200 Hz

        self.get_logger().info('Humanoid manager initialized')

    def joint_state_callback(self, msg):
        """Process joint state messages with history for velocity estimation."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                pos = msg.position[i]

                # Store in joint states
                self.joint_states[name] = {
                    'position': pos,
                    'velocity': 0.0,
                    'effort': 0.0
                }

                if i < len(msg.velocity):
                    self.joint_states[name]['velocity'] = msg.velocity[i]
                if i < len(msg.effort):
                    self.joint_states[name]['effort'] = msg.effort[i]

                # Store history for velocity estimation if needed
                if name not in self.joint_history:
                    self.joint_history[name] = deque(maxlen=self.history_length)

                self.joint_history[name].append((current_time, pos))

                # Estimate velocity if not provided by sensor
                if len(msg.velocity) <= i and len(self.joint_history[name]) >= 2:
                    # Calculate velocity from position history
                    old_time, old_pos = self.joint_history[name][0]
                    new_time, new_pos = self.joint_history[name][-1]
                    if new_time != old_time:
                        estimated_vel = (new_pos - old_pos) / (new_time - old_time)
                        self.joint_states[name]['velocity'] = estimated_vel

    def imu_callback(self, msg):
        """Process IMU data for balance and orientation."""
        self.imu_data = {
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }

    def pose_callback(self, msg):
        """Process robot pose information."""
        self.robot_pose = msg

    def request_stand_callback(self, request, response):
        """Handle stand request."""
        if self.safety_check():
            self.current_behavior = 'standing'
            self.execute_behavior('standing')
            response.data = 'Standing command accepted'
            self.get_logger().info('Standing command accepted')
        else:
            response.data = 'Safety check failed - standing command rejected'
            self.get_logger().warn('Standing command rejected due to safety check')

        return response

    def request_walk_callback(self, request, response):
        """Handle walk request."""
        if self.safety_check() and self.current_behavior != 'walking':
            self.current_behavior = 'walking'
            self.execute_behavior('walking')
            response.data = 'Walking command accepted'
            self.get_logger().info('Walking command accepted')
        else:
            response.data = 'Safety check failed or already walking - walking command rejected'
            self.get_logger().warn('Walking command rejected')

        return response

    def request_sit_callback(self, request, response):
        """Handle sit request."""
        if self.safety_check():
            self.current_behavior = 'sitting'
            self.execute_behavior('sitting')
            response.data = 'Sitting command accepted'
            self.get_logger().info('Sitting command accepted')
        else:
            response.data = 'Safety check failed - sitting command rejected'
            self.get_logger().warn('Sitting command rejected')

        return response

    def safety_check(self):
        """Perform safety checks before executing commands."""
        if not self.safety_enabled:
            return False

        # Check IMU data for dangerous accelerations
        if 'linear_acceleration' in self.imu_data:
            lin_acc = np.array(self.imu_data['linear_acceleration'])
            if np.linalg.norm(lin_acc) > self.safety_thresholds['imu_linear_acceleration']:
                self.get_logger().warn(f'Dangerous linear acceleration detected: {np.linalg.norm(lin_acc)}')
                return False

        # Check joint states for dangerous conditions
        for joint_name, state in self.joint_states.items():
            if abs(state['velocity']) > self.safety_thresholds['joint_velocity']:
                self.get_logger().warn(f'Dangerous velocity on joint {joint_name}: {state["velocity"]}')
                return False

            if abs(state['effort']) > self.safety_thresholds['joint_effort']:
                self.get_logger().warn(f'Dangerous effort on joint {joint_name}: {state["effort"]}')
                return False

        return True

    def execute_behavior(self, behavior):
        """Execute the specified behavior."""
        # This would call specific behavior controllers
        # For this example, we'll just send a command
        cmd_msg = Float64MultiArray()

        if behavior == 'standing':
            # Send commands for standing position
            cmd_msg.data = [0.0] * 12  # Example: 12 joints in standing position
        elif behavior == 'walking':
            # Send commands for walking gait
            cmd_msg.data = [0.1] * 12  # Example: 12 joints in walking position
        elif behavior == 'sitting':
            # Send commands for sitting position
            cmd_msg.data = [-0.1] * 12  # Example: 12 joints in sitting position

        self.command_pub.publish(cmd_msg)

    def main_control_loop(self):
        """Main control loop for humanoid behaviors."""
        # Update status
        status_msg = String()
        status_msg.data = f'Behavior: {self.current_behavior}, Safety: {self.safety_enabled}'
        self.status_pub.publish(status_msg)

        # Execute current behavior
        if self.current_behavior != 'idle' and self.safety_enabled:
            self.execute_behavior(self.current_behavior)

    def safety_monitor(self):
        """Continuous safety monitoring."""
        safety_status = Bool()

        # Perform safety checks
        is_safe = self.safety_check()
        safety_status.data = is_safe

        # Publish safety status
        self.safety_pub.publish(safety_status)

        # Log safety status changes
        if not is_safe and self.safety_enabled:
            self.get_logger().error('SAFETY VIOLATION DETECTED - EMERGENCY STOP')
            self.emergency_stop = True
            self.safety_enabled = False
            self.current_behavior = 'emergency_stop'
            self.execute_emergency_stop()


def main(args=None):
    rclpy.init(args=args)
    humanoid_manager = HumanoidManager()

    try:
        rclpy.spin(humanoid_manager)
    except KeyboardInterrupt:
        pass
    finally:
        humanoid_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Now let's implement a humanoid-specific inverse kinematics node:

```python
# my_robot_humanoid/my_robot_humanoid/humanoid_ik_solver.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time
import numpy as np
from scipy.spatial.transform import Rotation as R
import math


class HumanoidIKSolver(Node):
    def __init__(self):
        super().__init__('humanoid_ik_solver')

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Publishers
        self.left_hand_target_pub = self.create_publisher(
            Pose,
            '/left_hand_target',
            10)

        self.right_hand_target_pub = self.create_publisher(
            Pose,
            '/right_hand_target',
            10)

        self.left_foot_target_pub = self.create_publisher(
            Pose,
            '/left_foot_target',
            10)

        self.right_foot_target_pub = self.create_publisher(
            Pose,
            '/right_foot_target',
            10)

        self.ik_solution_pub = self.create_publisher(
            Float64MultiArray,
            '/ik_solution',
            10)

        # Services for IK requests
        self.left_hand_ik_srv = self.create_service(
            # Using standard service for example
            # In real system, would use custom service
            Float64MultiArray,
            'solve_left_hand_ik',
            self.solve_left_hand_ik)

        self.right_hand_ik_srv = self.create_service(
            Float64MultiArray,
            'solve_right_hand_ik',
            self.solve_right_hand_ik)

        # Internal state
        self.current_joint_positions = {}
        self.left_hand_pose = Pose()
        self.right_hand_pose = Pose()
        self.left_foot_pose = Pose()
        self.right_foot_pose = Pose()

        # Robot kinematic parameters (simplified for example)
        self.link_lengths = {
            'upper_arm': 0.3,  # meters
            'forearm': 0.25,
            'thigh': 0.4,
            'shin': 0.4,
            'torso': 0.6
        }

        # Timer for IK solution updates
        self.ik_timer = self.create_timer(0.05, self.ik_update_loop)  # 20 Hz

        self.get_logger().info('Humanoid IK solver initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def solve_left_hand_ik(self, request, response):
        """Solve inverse kinematics for left hand."""
        if len(request.data) >= 7:  # x, y, z, qx, qy, qz, qw
            target_pose = Pose()
            target_pose.position.x = request.data[0]
            target_pose.position.y = request.data[1]
            target_pose.position.z = request.data[2]
            target_pose.orientation.x = request.data[3]
            target_pose.orientation.y = request.data[4]
            target_pose.orientation.z = request.data[5]
            target_pose.orientation.w = request.data[6]

            # Store as target
            self.left_hand_pose = target_pose

            # Solve IK (simplified for example)
            solution = self.calculate_arm_ik(
                target_pose,
                side='left',
                current_positions=self.current_joint_positions
            )

            # Publish solution
            solution_msg = Float64MultiArray()
            solution_msg.data = solution
            self.ik_solution_pub.publish(solution_msg)

            response.data = [1.0]  # Success
        else:
            response.data = [0.0]  # Failure

        return response

    def solve_right_hand_ik(self, request, response):
        """Solve inverse kinematics for right hand."""
        if len(request.data) >= 7:  # x, y, z, qx, qy, qz, qw
            target_pose = Pose()
            target_pose.position.x = request.data[0]
            target_pose.position.y = request.data[1]
            target_pose.position.z = request.data[2]
            target_pose.orientation.x = request.data[3]
            target_pose.orientation.y = request.data[4]
            target_pose.orientation.z = request.data[5]
            target_pose.orientation.w = request.data[6]

            # Store as target
            self.right_hand_pose = target_pose

            # Solve IK (simplified for example)
            solution = self.calculate_arm_ik(
                target_pose,
                side='right',
                current_positions=self.current_joint_positions
            )

            # Publish solution
            solution_msg = Float64MultiArray()
            solution_msg.data = solution
            self.ik_solution_pub.publish(solution_msg)

            response.data = [1.0]  # Success
        else:
            response.data = [0.0]  # Failure

        return response

    def calculate_arm_ik(self, target_pose, side='left', current_positions=None):
        """
        Simplified inverse kinematics for humanoid arm.
        In a real implementation, this would use more sophisticated algorithms.
        """
        # Extract target position
        target_pos = np.array([
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        ])

        # Calculate basic arm IK (simplified)
        # This is a placeholder - real IK would be much more complex
        solution = []

        # For example, return some joint angles based on target position
        # In reality, this would solve the full kinematic chain
        if side == 'left':
            # Example joint positions for left arm
            solution = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # Placeholder values
        else:  # right
            # Example joint positions for right arm
            solution = [-0.1, 0.2, -0.3, 0.4, -0.5, 0.6]  # Placeholder values

        return solution

    def calculate_leg_ik(self, target_pose, side='left'):
        """
        Simplified inverse kinematics for humanoid leg.
        """
        # Extract target position
        target_pos = np.array([
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        ])

        # Calculate basic leg IK (simplified)
        # This is a placeholder - real IK would be much more complex
        solution = []

        # For example, return some joint angles based on target position
        if side == 'left':
            # Example joint positions for left leg
            solution = [0.0, -0.5, 0.5, 0.0, 0.0, 0.0]  # Placeholder values
        else:  # right
            # Example joint positions for right leg
            solution = [0.0, -0.5, 0.5, 0.0, 0.0, 0.0]  # Placeholder values

        return solution

    def ik_update_loop(self):
        """Update IK solutions and publish targets."""
        # Publish current targets for visualization
        self.left_hand_target_pub.publish(self.left_hand_pose)
        self.right_hand_target_pub.publish(self.right_hand_pose)
        self.left_foot_target_pub.publish(self.left_foot_pose)
        self.right_foot_target_pub.publish(self.right_foot_pose)


def main(args=None):
    rclpy.init(args=args)
    ik_solver = HumanoidIKSolver()

    try:
        rclpy.spin(ik_solver)
    except KeyboardInterrupt:
        pass
    finally:
        ik_solver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Hardware/GPU Notes

Humanoid-specific ROS 2 implementations have demanding hardware requirements:

- **Real-time Kernel**: Use real-time kernel patches for deterministic behavior
- **Multi-core Processing**: Distribute different control tasks across CPU cores
- **GPU Acceleration**: Use for inverse kinematics, computer vision, and other intensive computations
- **Low-latency Communication**: Use appropriate QoS settings and network configuration
- **Safety Processors**: Dedicated safety systems for emergency stopping

For NVIDIA Jetson platforms, leverage the GPU for vision processing and use the real-time capabilities of the ARM cores for control.

## Simulation Path

Testing humanoid-specific ROS 2 systems in simulation:

```bash
# Terminal 1: Start humanoid robot simulation in Gazebo
ros2 launch my_robot_gazebo op3_world.launch.py

# Terminal 2: Start the humanoid manager
ros2 run my_robot_humanoid humanoid_manager

# Terminal 3: Start the IK solver
ros2 run my_robot_humanoid humanoid_ik_solver

# Terminal 4: Send commands to the robot
ros2 service call /request_stand std_msgs/srv/String "{data: 'stand'}"

# Terminal 5: Monitor the system
ros2 topic echo /humanoid_status
ros2 topic echo /safety_status

# Terminal 6: Send IK requests
ros2 service call /solve_left_hand_ik example_interfaces/srv/Float64MultiArray "{data: [0.3, 0.2, 0.8, 0.0, 0.0, 0.0, 1.0]}"
```

Simulation allows for safe testing of complex humanoid behaviors before hardware deployment.

## Real-World Path

For real hardware deployment:

1. **Safety First**: Ensure all safety systems are operational before enabling hardware
2. **Calibration**: Calibrate all sensors and actuators to match simulation
3. **Gradual Testing**: Start with simple movements, increase complexity gradually
4. **Monitoring**: Continuous monitoring of all safety parameters
5. **Fallback Systems**: Ensure safe fallback behaviors when control fails

Example of a safety-aware humanoid system:

```python
# my_robot_safety/my_robot_safety/humanoid_safety_manager.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, WrenchStamped
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Time
import numpy as np
import threading


class HumanoidSafetyManager(Node):
    def __init__(self):
        super().__init__('humanoid_safety_manager')

        # Subscribers for all safety-critical data
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)

        self.force_sub = self.create_subscription(
            # In a real system, this would be WrenchStamped
            # For this example, using Float64MultiArray
            Float64MultiArray,
            '/force_torque',
            self.force_callback,
            10)

        # Publishers for safety status
        self.safety_status_pub = self.create_publisher(
            Bool,
            '/safety_status',
            10)

        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10)

        self.safety_log_pub = self.create_publisher(
            String,
            '/safety_log',
            10)

        # Internal state
        self.joint_states = {}
        self.imu_data = {}
        self.force_data = {}
        self.safety_enabled = True
        self.emergency_stop_active = False
        self.safety_violations = []

        # Safety thresholds
        self.safety_thresholds = {
            'imu_angular_velocity_max': 5.0,  # rad/s
            'imu_linear_acceleration_max': 20.0,  # m/s^2
            'imu_orientation_threshold': 0.5,  # rad from upright
            'joint_position_min': -3.0,  # rad
            'joint_position_max': 3.0,   # rad
            'joint_velocity_max': 8.0,   # rad/s
            'joint_effort_max': 50.0,    # Nm
            'force_threshold': 100.0,    # N
        }

        # Timer for safety monitoring
        self.safety_timer = self.create_timer(0.001, self.safety_monitor)  # 1000 Hz

        # Timer for safety status publishing
        self.status_timer = self.create_timer(0.1, self.publish_safety_status)  # 10 Hz

        self.get_logger().info('Humanoid safety manager initialized')

    def joint_state_callback(self, msg):
        """Process joint state messages for safety monitoring."""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                pos = msg.position[i]
                vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
                effort = msg.effort[i] if i < len(msg.effort) else 0.0

                self.joint_states[name] = {
                    'position': pos,
                    'velocity': vel,
                    'effort': effort
                }

    def imu_callback(self, msg):
        """Process IMU data for orientation and acceleration safety."""
        self.imu_data = {
            'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
            'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        }

    def force_callback(self, msg):
        """Process force/torque data."""
        if len(msg.data) >= 6:  # [force_x, force_y, force_z, torque_x, torque_y, torque_z]
            self.force_data = {
                'force': msg.data[0:3],
                'torque': msg.data[3:6]
            }

    def safety_monitor(self):
        """Continuous safety monitoring."""
        violations = []

        # Check IMU data
        if self.imu_data:
            # Check angular velocity
            ang_vel = np.array(self.imu_data['angular_velocity'])
            if np.linalg.norm(ang_vel) > self.safety_thresholds['imu_angular_velocity_max']:
                violations.append(f'High angular velocity: {np.linalg.norm(ang_vel):.2f}')

            # Check linear acceleration
            lin_acc = np.array(self.imu_data['linear_acceleration'])
            if np.linalg.norm(lin_acc) > self.safety_thresholds['imu_linear_acceleration_max']:
                violations.append(f'High linear acceleration: {np.linalg.norm(lin_acc):.2f}')

            # Check orientation (simplified - check if robot is roughly upright)
            orientation = self.imu_data['orientation']
            # Convert quaternion to rotation matrix and check z-axis
            # Simplified check for this example
            if abs(orientation[2]) > 0.7:  # If z-component of orientation is too large
                violations.append(f'Unstable orientation: {orientation}')

        # Check joint states
        for joint_name, state in self.joint_states.items():
            if state['position'] < self.safety_thresholds['joint_position_min']:
                violations.append(f'Joint {joint_name} position too low: {state["position"]:.2f}')
            elif state['position'] > self.safety_thresholds['joint_position_max']:
                violations.append(f'Joint {joint_name} position too high: {state["position"]:.2f}')

            if abs(state['velocity']) > self.safety_thresholds['joint_velocity_max']:
                violations.append(f'Joint {joint_name} velocity too high: {state["velocity"]:.2f}')

            if abs(state['effort']) > self.safety_thresholds['joint_effort_max']:
                violations.append(f'Joint {joint_name} effort too high: {state["effort"]:.2f}')

        # Check force data
        if self.force_data:
            force_mag = np.linalg.norm(self.force_data['force'])
            if force_mag > self.safety_thresholds['force_threshold']:
                violations.append(f'High force detected: {force_mag:.2f}')

        # Update violations and check for emergency stop
        self.safety_violations = violations

        if violations:
            # Log violations
            for violation in violations:
                self.get_logger().warn(f'Safety violation: {violation}')
                log_msg = String()
                log_msg.data = f'Safety violation: {violation}'
                self.safety_log_pub.publish(log_msg)

            # Trigger emergency stop if critical violations
            if self.is_critical_violation(violations):
                self.trigger_emergency_stop()

    def is_critical_violation(self, violations):
        """Determine if any violations are critical enough for emergency stop."""
        critical_keywords = ['acceleration', 'orientation', 'force']
        for violation in violations:
            if any(keyword in violation.lower() for keyword in critical_keywords):
                return True
        return False

    def trigger_emergency_stop(self):
        """Trigger emergency stop procedure."""
        if not self.emergency_stop_active:
            self.get_logger().error('EMERGENCY STOP TRIGGERED')
            self.emergency_stop_active = True
            self.safety_enabled = False

            # Publish emergency stop command
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)

            # Log emergency stop
            log_msg = String()
            log_msg.data = 'EMERGENCY STOP ACTIVATED'
            self.safety_log_pub.publish(log_msg)

    def publish_safety_status(self):
        """Publish current safety status."""
        status_msg = Bool()
        status_msg.data = self.safety_enabled and not self.emergency_stop_active
        self.safety_status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    safety_manager = HumanoidSafetyManager()

    try:
        rclpy.spin(safety_manager)
    except KeyboardInterrupt:
        pass
    finally:
        safety_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Spec-Build-Test Checklist

- [ ] Verify humanoid-specific ROS 2 nodes initialize correctly
- [ ] Confirm safety systems monitor all critical parameters
- [ ] Test inverse kinematics solutions for accuracy
- [ ] Validate real-time performance requirements
- [ ] Check proper error handling and fallback behaviors
- [ ] Verify emergency stop functionality

## APA Citations

- Kuffner, J., Nishiwaki, K., Kagami, S., Inaba, M., & Inoue, H. (2004). Motion planning for humanoid robots. *Robotics Research*, 365-374.
- Nakanishi, J., Cory, R., Mistry, M., Peters, J., & Schaal, S. (2008). Operational space control: A theoretical and empirical comparison. *The International Journal of Robotics Research*, 27(6), 737-757.
- Wensing, P. M., & Orin, D. E. (2013). Improved computation of the Jacobian matrices for inverse dynamics in robotics. *The International Journal of Robotics Research*, 32(10), 1225-1235.
- Pratt, J., & Pratt, G. (1998). Intuitive control of a planar bipedal walking robot. *Proceedings of the 1998 IEEE International Conference on Robotics and Automation*, 2, 1458-1465.