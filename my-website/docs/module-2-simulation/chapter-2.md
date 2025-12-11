---
title: Chapter 9 - Gazebo for ROS 2 Integration
sidebar_position: 9
---

# Chapter 9: Gazebo for ROS 2 Integration

## Why This Concept Matters for Humanoids

Gazebo integration with ROS 2 is fundamental for humanoid robotics development, providing a bridge between high-level control algorithms and realistic physics simulation. For humanoid robots, which require precise coordination of multiple joints and sensors, Gazebo enables comprehensive testing of control strategies, gait patterns, and balance algorithms in a safe environment. The integration allows developers to validate complex multi-joint controllers, sensor fusion algorithms, and whole-body behaviors before risking expensive hardware. This integration is particularly critical for humanoid robots because of their complexity and the safety requirements involved in physical testing.

## Theory

Gazebo-ROS 2 integration involves several key components that work together to create a comprehensive simulation environment:

### Gazebo-ROS Bridge Architecture
The integration layer includes:
- **Gazebo ROS packages**: Core packages that enable ROS 2 communication within Gazebo
- **Plugin system**: Mechanism for extending Gazebo functionality with ROS 2 interfaces
- **Message passing**: Standardized interfaces for sensor data and actuator commands
- **Clock synchronization**: Simulation time coordination between ROS 2 nodes and Gazebo

### Communication Patterns
Gazebo-ROS 2 uses specific communication patterns:
- **Sensor publishers**: Real-time sensor data streams (camera, IMU, joint states)
- **Actuator subscribers**: Command inputs for joint control and other actuators
- **Service interfaces**: One-time configuration and control operations
- **Action interfaces**: Long-running operations with feedback (navigation, manipulation)

### Physics and Control Integration
The integration handles:
- **Real-time control loops**: Maintaining specified control frequencies
- **Physics accuracy**: Balancing simulation fidelity with performance
- **Sensor simulation**: Modeling sensor characteristics and noise
- **Multi-robot coordination**: Handling multiple robots in shared environments

## Implementation

Let's implement a complete Gazebo-ROS 2 integration for a humanoid robot:

### URDF Model with Gazebo Extensions

```xml
<?xml version="1.0" ?>
<robot name="humanoid_with_gazebo" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.1"/>
      <geometry>
        <box size="0.3 0.2 0.2"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.1"/>
      <geometry>
        <box size="0.3 0.2 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0.2"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2"/>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.2"/>
      <geometry>
        <box size="0.2 0.15 0.4"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.2"/>
  </joint>

  <!-- Head with camera -->
  <link name="head">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0.05"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.05"/>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.05"/>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="2.0"/>
  </joint>

  <!-- Left arm -->
  <link name="left_shoulder">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 -0.075"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.075"/>
      <geometry>
        <cylinder length="0.15" radius="0.03"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075"/>
      <geometry>
        <cylinder length="0.15" radius="0.03"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_shoulder"/>
    <origin xyz="0.05 0.08 0.2"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="2.0"/>
  </joint>

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find my_humanoid_description)/config/humanoid_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Gazebo material and visual properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="torso">
    <material>Gazebo/Blue</material>
  </gazebo>

  <!-- Camera sensor on head -->
  <gazebo reference="head">
    <sensor name="camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>head_camera_optical_frame</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>10.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU sensor on torso -->
  <gazebo reference="torso">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
        <frame_name>torso_imu_frame</frame_name>
        <body_name>torso</body_name>
        <topic>__default_topic__</topic>
        <serviceName>__default_service_name__</serviceName>
        <gaussianNoise>0.001</gaussianNoise>
        <updateRateHZ>100.0</updateRateHZ>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### Controller Configuration

```yaml
# config/humanoid_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # Position controllers for joints
    left_hip_position_controller:
      type: position_controllers/JointPositionController

    right_hip_position_controller:
      type: position_controllers/JointPositionController

    left_knee_position_controller:
      type: position_controllers/JointPositionController

    right_knee_position_controller:
      type: position_controllers/JointPositionController

    left_ankle_position_controller:
      type: position_controllers/JointPositionController

    right_ankle_position_controller:
      type: position_controllers/JointPositionController

# Individual joint controller parameters
left_hip_position_controller:
  ros__parameters:
    joints:
      - left_hip_joint
    interface_name: position

right_hip_position_controller:
  ros__parameters:
    joints:
      - right_hip_joint
    interface_name: position

left_knee_position_controller:
  ros__parameters:
    joints:
      - left_knee_joint
    interface_name: position

right_knee_position_controller:
  ros__parameters:
    joints:
      - right_knee_joint
    interface_name: position

left_ankle_position_controller:
  ros__parameters:
    joints:
      - left_ankle_joint
    interface_name: position

right_ankle_position_controller:
  ros__parameters:
    joints:
      - right_ankle_joint
    interface_name: position
```

### Launch File for Gazebo Integration

```python
#!/usr/bin/env python3
# launch/humanoid_gazebo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description_path = LaunchConfiguration('robot_description_path')
    world_path = LaunchConfiguration('world_path')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_robot_description_path = DeclareLaunchArgument(
        'robot_description_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_humanoid_description'),
            'urdf',
            'humanoid_with_gazebo.urdf.xacro'
        ]),
        description='Path to robot URDF file'
    )

    declare_world_path = DeclareLaunchArgument(
        'world_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_humanoid_gazebo'),
            'worlds',
            'simple_room.world'
        ]),
        description='Path to Gazebo world file'
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            LaunchConfiguration('robot_description_path'),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    # Spawn robot in Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_path,
            'verbose': 'false',
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
        ],
        output='screen',
    )

    # Load and activate controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_position_controllers = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'left_hip_position_controller'],
        output='screen'
    )

    # Create launch description
    ld = LaunchDescription()

    # Add declared launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_robot_description_path)
    ld.add_action(declare_world_path)

    # Add actions
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    # Add controller loading after spawn
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    ))

    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_position_controllers],
        )
    ))

    return ld
```

### Control Node for Humanoid Robot

```python
#!/usr/bin/env python3
# scripts/humanoid_control_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
import numpy as np
import time


class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Declare parameters
        self.declare_parameter('control_frequency', 100.0)
        self.declare_parameter('max_torque', 100.0)
        self.declare_parameter('joint_names', [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ])

        # Get parameters
        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_torque = self.get_parameter('max_torque').value
        self.joint_names = self.get_parameter('joint_names').value

        # Publishers for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10
        )

        # Subscribers for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer for control loop
        self.control_timer = self.create_timer(
            1.0/self.control_frequency,
            self.control_loop
        )

        # Internal state
        self.current_joint_states = JointState()
        self.balance_controller = BalanceController()
        self.trajectory_generator = TrajectoryGenerator()

        self.get_logger().info(f'Humanoid Controller initialized with {len(self.joint_names)} joints')

    def joint_state_callback(self, msg):
        """Update current joint states"""
        self.current_joint_states = msg

    def control_loop(self):
        """Main control loop"""
        # Get current joint positions
        current_positions = self.get_current_positions()

        # Generate balance control commands
        balance_commands = self.balance_controller.compute_balance_control(
            current_positions
        )

        # Generate trajectory commands
        trajectory_commands = self.trajectory_generator.generate_trajectory(
            current_positions
        )

        # Combine commands
        final_commands = self.combine_commands(
            balance_commands,
            trajectory_commands
        )

        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = final_commands
        self.joint_cmd_pub.publish(cmd_msg)

        self.get_logger().debug(f'Published joint commands: {final_commands}')

    def get_current_positions(self):
        """Extract current joint positions from joint state message"""
        positions = []
        for joint_name in self.joint_names:
            try:
                idx = self.current_joint_states.name.index(joint_name)
                positions.append(self.current_joint_states.position[idx])
            except ValueError:
                positions.append(0.0)  # Default if joint not found
        return positions

    def combine_commands(self, balance_cmd, trajectory_cmd):
        """Combine balance and trajectory commands"""
        combined = []
        for b, t in zip(balance_cmd, trajectory_cmd):
            # Simple weighted combination (can be more sophisticated)
            combined.append(0.7 * b + 0.3 * t)
        return combined


class BalanceController:
    """Simple balance controller for humanoid robot"""

    def __init__(self):
        self.kp = 10.0  # Proportional gain
        self.kd = 1.0   # Derivative gain

    def compute_balance_control(self, current_positions):
        """Compute balance control commands based on current positions"""
        commands = []
        for pos in current_positions:
            # Simple PD control to maintain neutral position
            error = 0.0 - pos  # Target is 0.0 for neutral
            command = self.kp * error  # Proportional control
            commands.append(command)
        return commands


class TrajectoryGenerator:
    """Generate joint trajectories for humanoid movement"""

    def __init__(self):
        self.time_counter = 0.0
        self.frequency = 0.1  # Hz for movement patterns

    def generate_trajectory(self, current_positions):
        """Generate periodic trajectory for demonstration"""
        self.time_counter += 0.01  # Increment based on control frequency

        commands = []
        for i, pos in enumerate(current_positions):
            # Generate different patterns for different joints
            if i < 3:  # Left leg
                command = 0.2 * np.sin(2 * np.pi * self.frequency * self.time_counter + i * np.pi/3)
            else:  # Right leg
                command = 0.2 * np.sin(2 * np.pi * self.frequency * self.time_counter + i * np.pi/3 + np.pi)
            commands.append(command)

        return commands


def main(args=None):
    rclpy.init(args=args)

    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down humanoid controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Hardware/GPU Notes

Gazebo-ROS 2 integration for humanoid robotics has specific hardware requirements:

### Minimum Requirements
- **CPU**: Quad-core processor (8+ cores recommended for complex humanoid models)
- **Memory**: 8GB RAM minimum, 16GB+ recommended for multi-joint humanoid simulation
- **GPU**: OpenGL 3.3+ compatible graphics card
- **Storage**: SSD recommended for faster asset loading

### Recommended Specifications for Humanoid Simulation
- **CPU**: 12+ cores for real-time physics simulation of complex humanoid models
- **Memory**: 32GB+ for detailed humanoid models with multiple DOF and sensors
- **GPU**: NVIDIA RTX 4070 Ti or equivalent with CUDA support for advanced sensor simulation
- **VRAM**: 12GB+ for detailed visual simulation (cameras, rendering)

### Performance Considerations
- Physics complexity scales with number of joints and contacts
- Sensor simulation (especially cameras) is GPU-intensive
- Real-time performance requires careful optimization of collision meshes
- Multi-robot simulation increases computational requirements exponentially

## Simulation Path

For developing Gazebo-ROS 2 integration for humanoid robotics:

### Initial Setup
1. Install Gazebo Garden/Fortress with ROS 2 Humble integration
2. Create URDF model with Gazebo extensions for your humanoid
3. Configure physics parameters for realistic humanoid simulation
4. Set up sensor simulation (cameras, IMUs, joint encoders)

### Basic Integration
1. Test URDF model loading in Gazebo
2. Verify ROS 2 communication with joint states
3. Implement basic joint control
4. Validate sensor data output

### Advanced Integration
1. Add complex humanoid controllers
2. Implement balance and locomotion algorithms
3. Add multiple sensors for perception
4. Test whole-body control strategies

### Validation Process
1. Compare simulation vs. real-world behavior (when available)
2. Adjust physics parameters for better fidelity
3. Validate control strategies in simulation
4. Prepare for hardware deployment

## Real-World Path

Transitioning from Gazebo simulation to real hardware:

### Pre-deployment Validation
1. Test all safety systems in simulation first
2. Validate emergency stop procedures in simulation
3. Verify communication protocols work in simulation
4. Check sensor accuracy and noise models match hardware

### Hardware Integration
1. Map simulation parameters to real hardware capabilities
2. Adjust control gains for real-world performance
3. Implement hardware-specific interfaces and safety systems
4. Validate sensor calibration against simulation models

### Deployment Strategy
1. Start with simple behaviors in safe environment
2. Gradually increase complexity and range of motion
3. Monitor system performance and safety metrics
4. Iterate based on real-world observations

### Safety Considerations
1. Implement multiple safety layers in both simulation and hardware
2. Ensure reliable emergency stop mechanisms
3. Monitor for unexpected behaviors in both domains
4. Maintain human oversight during initial deployment

## Spec-Build-Test Checklist

- [ ] URDF model loads correctly in Gazebo with all Gazebo extensions
- [ ] Physics parameters are realistic for humanoid dynamics
- [ ] Joint limits and constraints are properly configured
- [ ] Sensor simulation matches real hardware characteristics
- [ ] ROS 2 communication works for all interfaces
- [ ] Control algorithms work in simulation environment
- [ ] Collision detection works properly with humanoid morphology
- [ ] Simulation runs at real-time speed or faster with humanoid model
- [ ] Multi-joint control strategies are validated
- [ ] Emergency stop procedures are validated in simulation
- [ ] Performance metrics are monitored during simulation
- [ ] Simulation-to-reality transfer parameters are documented
- [ ] All controller dependencies are properly declared
- [ ] Launch files correctly initialize the simulation environment

## APA Citations

- Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 2350-2354.
- Godoy, J., Mandry, N., & Remirez, A. (2018). Gazebo: A 3D multi-robot simulator. *Gazebo ROS Packages Documentation*.
- Tedrake, R., Jackowski, Z., Miller, R., Murphey, J., & Erez, T. (2010). Using system identification to obtain reliable models for legged robots. *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 1417-1423.
- Coumans, E., & Bai, Y. (2016). Mujoco: A physics engine for model-based control. *IEEE International Conference on Robotics and Automation*.
- Murai, R., & Kyrki, V. (2021). Simulation to reality transfer in robotics: A survey. *IEEE Access*, 9, 142395-142418.