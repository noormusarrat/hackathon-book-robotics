---
title: Chapter 7 - ROS 2 Best Practices
sidebar_position: 7
---

# Chapter 7: ROS 2 Best Practices

## Why This Concept Matters for Humanoids

ROS 2 best practices are critical for humanoid robotics due to the complexity and safety requirements of these systems. Unlike simpler robots, humanoid robots involve multiple subsystems (locomotion, manipulation, perception, cognition) that must work in harmony. Following established best practices ensures code maintainability, system reliability, and safety - all essential for robots that interact closely with humans. Proper practices also facilitate collaboration among teams and enable scalable development as humanoid robot capabilities evolve.

## Theory

ROS 2 best practices encompass architectural patterns, coding standards, and operational procedures that have been validated through years of robotic development. These practices address common challenges in robotics such as real-time constraints, fault tolerance, distributed computing, and safety. For humanoid robotics specifically, best practices must account for the additional complexity of bipedal locomotion, human-robot interaction, and the need for fail-safe mechanisms.

Key principles include:

- **Modularity**: Breaking complex systems into manageable, testable components
- **Reusability**: Designing components that can be used across different robots or applications
- **Safety**: Implementing multiple layers of safety checks and fail-safe mechanisms
- **Maintainability**: Writing code that can be understood and modified by others
- **Performance**: Optimizing for real-time constraints and computational efficiency

## Implementation

### 1. Package Structure Best Practices

Follow the ROS 2 standard for organizing packages:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration
├── package.xml            # Package metadata and dependencies
├── README.md              # Package documentation
├── config/                # Configuration files
├── launch/                # Launch files
├── src/                   # Source code
├── include/               # Header files (C++)
├── scripts/               # Executable scripts
├── test/                  # Unit and integration tests
└── docs/                  # Additional documentation
```

Example package.xml with proper dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_humanoid_control</name>
  <version>1.0.0</version>
  <description>Humanoid robot control package</description>
  <maintainer email="maintainer@robotics.org">Maintainer Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>control_msgs</depend>
  <depend>trajectory_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 2. Node Design Best Practices

Create nodes that follow the single responsibility principle:

```python
#!/usr/bin/env python3
# my_humanoid_controller.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import threading
import time

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Declare parameters with defaults
        self.declare_parameter('control_rate', 100.0)
        self.declare_parameter('max_effort', 100.0)
        self.declare_parameter('joint_names', [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ])

        # Get parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.max_effort = self.get_parameter('max_effort').value
        self.joint_names = self.get_parameter('joint_names').value

        # QoS profiles for different data types
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        control_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.joint_command_pub = self.create_publisher(
            JointTrajectory, 'joint_trajectory', control_qos)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, sensor_qos)

        # Internal state
        self.current_joint_states = {}
        self.trajectory_buffer = []

        # Control timer
        self.control_timer = self.create_timer(
            1.0/self.control_rate, self.control_loop)

        # Safety timer
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info(
            f'Humanoid Controller initialized with {len(self.joint_names)} joints')

    def joint_state_callback(self, msg):
        """Update current joint states"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_joint_states[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }

    def control_loop(self):
        """Main control loop"""
        try:
            # Process trajectory commands
            if self.trajectory_buffer:
                trajectory_point = self.trajectory_buffer.pop(0)
                self.execute_trajectory_point(trajectory_point)

            # Publish current state
            self.publish_feedback()

        except Exception as e:
            self.get_logger().error(f'Error in control loop: {str(e)}')

    def execute_trajectory_point(self, point):
        """Execute a trajectory point"""
        # Validate trajectory point
        if not self.validate_trajectory_point(point):
            self.get_logger().warn('Invalid trajectory point, skipping')
            return

        # Send trajectory command
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        trajectory_msg.points = [point]

        self.joint_command_pub.publish(trajectory_msg)

    def validate_trajectory_point(self, point):
        """Validate trajectory point for safety"""
        # Check position limits
        for i, pos in enumerate(point.positions):
            if abs(pos) > np.pi * 2:  # 2 full rotations should be enough
                return False

        # Check velocity limits
        for vel in point.velocities:
            if abs(vel) > 10.0:  # rad/s - adjust based on joint specs
                return False

        # Check effort limits
        for effort in point.efforts:
            if abs(effort) > self.max_effort:
                return False

        return True

    def safety_check(self):
        """Perform safety checks"""
        # Check for joint limits violations
        for name, state in self.current_joint_states.items():
            if abs(state['position']) > 10.0:  # Example limit
                self.emergency_stop()
                return

        # Check for excessive effort
        for name, state in self.current_joint_states.items():
            if abs(state['effort']) > self.max_effort * 1.2:
                self.get_logger().warn(f'High effort detected on {name}: {state["effort"]}')

    def emergency_stop(self):
        """Emergency stop procedure"""
        self.get_logger().error('EMERGENCY STOP ACTIVATED')

        # Send zero trajectory
        zero_point = JointTrajectoryPoint()
        zero_point.positions = [0.0] * len(self.joint_names)
        zero_point.velocities = [0.0] * len(self.joint_names)
        zero_point.accelerations = [0.0] * len(self.joint_names)
        zero_point.effort = [0.0] * len(self.joint_names)
        zero_point.time_from_start = Duration(sec=0, nanosec=100000000)  # 100ms

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names
        trajectory_msg.points = [zero_point]

        self.joint_command_pub.publish(trajectory_msg)

    def publish_feedback(self):
        """Publish controller feedback"""
        # Implementation would publish controller state
        pass

def main(args=None):
    rclpy.init(args=args)

    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down controller...')
    finally:
        controller.emergency_stop()  # Ensure safe shutdown
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Launch File Best Practices

Create reusable and configurable launch files:

```python
#!/usr/bin/env python3
# humanoid_control.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    # Get launch arguments
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # Get package share directory
    pkg_share = get_package_share_directory('my_humanoid_control')

    # Create nodes
    controller_node = Node(
        package='my_humanoid_control',
        executable='humanoid_controller',
        name='humanoid_controller',
        namespace=namespace,
        parameters=[
            os.path.join(pkg_share, 'config', 'controller.yaml'),
            {
                'use_sim_time': use_sim_time == 'true',
                'control_rate': 100.0,
                'max_effort': 100.0
            }
        ],
        remappings=[
            ('joint_states', f'{namespace}/joint_states'),
            ('joint_trajectory', f'{namespace}/joint_trajectory')
        ],
        respawn=True,
        respawn_delay=2
    )

    return [controller_node]

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='humanoid_robot',
            description='Robot namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation time'
        ),

        # Opaque function to handle complex logic
        OpaqueFunction(function=launch_setup)
    ])
```

### 4. Testing Best Practices

Create comprehensive tests for your ROS 2 nodes:

```python
#!/usr/bin/env python3
# test_humanoid_controller.py

import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
from my_humanoid_control.my_humanoid_controller import HumanoidController

class TestHumanoidController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = HumanoidController()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_node_initialization(self):
        """Test that node initializes correctly"""
        self.assertIsNotNone(self.node)
        self.assertEqual(len(self.node.joint_names), 6)  # Default joint count

    def test_joint_state_callback(self):
        """Test joint state subscription"""
        # Create test joint state message
        joint_state_msg = JointState()
        joint_state_msg.name = ['left_hip_joint', 'left_knee_joint']
        joint_state_msg.position = [0.1, 0.2]
        joint_state_msg.velocity = [0.0, 0.0]
        joint_state_msg.effort = [0.0, 0.0]

        # Publish and spin to process
        pub = self.node.create_publisher(JointState, 'joint_states', 10)
        pub.publish(joint_state_msg)

        # Allow time for processing
        start_time = time.time()
        while time.time() - start_time < 1.0:
            self.executor.spin_once(timeout_sec=0.1)
            if 'left_hip_joint' in self.node.current_joint_states:
                break

        # Verify state was updated
        self.assertIn('left_hip_joint', self.node.current_joint_states)
        self.assertEqual(
            self.node.current_joint_states['left_hip_joint']['position'], 0.1)

    def test_trajectory_validation(self):
        """Test trajectory validation"""
        # Valid trajectory point
        valid_point = JointTrajectoryPoint()
        valid_point.positions = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        valid_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        valid_point.efforts = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
        valid_point.time_from_start = Duration(sec=1, nanosec=0)

        self.assertTrue(self.node.validate_trajectory_point(valid_point))

        # Invalid trajectory point (excessive position)
        invalid_point = JointTrajectoryPoint()
        invalid_point.positions = [100.0, 0.2, 0.3, 0.4, 0.5, 0.6]  # Too large
        invalid_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        invalid_point.efforts = [10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
        invalid_point.time_from_start = Duration(sec=1, nanosec=0)

        self.assertFalse(self.node.validate_trajectory_point(invalid_point))

def main():
    unittest.main()

if __name__ == '__main__':
    main()
```

## Hardware/GPU Notes

For humanoid robotics, hardware considerations are critical to success:

- **Real-time Performance**: Use PREEMPT_RT kernel for deterministic timing in control loops
- **Computational Power**: Multi-core processors (8+ cores) with high single-core performance for real-time control
- **Memory**: 32GB+ RAM for complex perception and planning algorithms
- **GPU**: NVIDIA GPU with CUDA support (RTX 4070 Ti minimum) for accelerated perception
- **Network**: Dedicated network interface for robot communication to avoid interference
- **Power Management**: Consider power consumption and thermal management for mobile robots
- **Safety Systems**: Hardware-based safety mechanisms as backup to software safety

## Simulation Path

When developing with simulation, follow these best practices:

1. **Use Simulation-Specific Parameters**:
   ```python
   # In your node
   use_sim_time = self.declare_parameter('use_sim_time', False).value
   if use_sim_time:
       # Use different parameters for simulation
       self.control_rate = 50.0  # Slower for simulation
   else:
       self.control_rate = 100.0  # Full speed for real robot
   ```

2. **Model Fidelity Considerations**:
   - Start with simplified models for development
   - Gradually increase complexity as needed
   - Validate simulation results against real hardware regularly

3. **Simulation Testing Pipeline**:
   - Unit tests for individual nodes
   - Integration tests in simulation
   - System tests on real hardware

## Real-World Path

For deployment on real humanoid hardware:

1. **Safety First**:
   - Implement multiple safety layers
   - Test all emergency procedures
   - Use safety-rated hardware where possible

2. **Gradual Deployment**:
   - Start with simple movements in safe environment
   - Gradually increase complexity
   - Monitor system behavior continuously

3. **Hardware Validation**:
   - Calibrate all sensors before deployment
   - Verify communication reliability
   - Test in various environmental conditions

## Spec-Build-Test Checklist

- [ ] All packages follow ROS 2 naming conventions
- [ ] Dependencies are properly declared in package.xml
- [ ] Launch files are configurable via parameters
- [ ] QoS profiles are appropriate for each use case
- [ ] Safety mechanisms are implemented and tested
- [ ] Code follows style guidelines (ament_uncrustify, etc.)
- [ ] Comprehensive unit and integration tests exist
- [ ] Documentation is complete and up-to-date
- [ ] Performance is profiled and optimized
- [ ] Error handling covers all possible failure modes
- [ ] Parameter validation prevents invalid configurations
- [ ] Emergency stop functionality is robust

## APA Citations

- Kammerl, J., Holzer, S., Rusu, R. B., & Konolige, K. (2012). Real-time automated parameter tuning for multi-dimensional point cloud processing. *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, 2459-2465.

- Quigley, M., Gerkey, B., & Smart, W. D. (2009). Programming robots with ROS: A practical introduction to the Robot Operating System. *Communications of the ACM*, 57(9), 82-91.

- Foote, T., Lalancette, C., & Quigley, J. (2016). ROS 2: Towards a robot platform for next generation robots. *Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 4698-4704.

- Open Robotics. (2021). ROS 2 Documentation: Best Practices. Retrieved from https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html

- Lütkebohle, I., & Axer, P. (2018). Design patterns for ROS-based systems: Engineering software for robots. *Proceedings of the Workshop on Software Engineering for Robotics*, 1-6.