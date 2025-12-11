---
title: Chapter 10 - Physics and Sensors in Gazebo
sidebar_position: 10
---

# Chapter 10: Physics and Sensors in Gazebo

## Why This Concept Matters for Humanoids

Accurate physics simulation and realistic sensor modeling are crucial for humanoid robotics, as these robots must interact with the physical world in complex ways. Humanoid robots rely on precise balance, coordinated movement, and environmental interaction, all of which depend heavily on accurate physics simulation. For safe and effective humanoid operation, the simulation must accurately model contact forces, friction, and multi-body dynamics. Similarly, sensor simulation must closely match real hardware to ensure successful simulation-to-reality transfer. Without accurate physics and sensor simulation, control algorithms developed in simulation may fail when deployed on real hardware, potentially causing damage or safety issues.

## Theory

Physics simulation in Gazebo involves modeling the fundamental laws of physics to create realistic robot-environment interactions:

### Physics Engine Fundamentals
Gazebo uses Open Dynamics Engine (ODE), Bullet, or DART physics engines to simulate:
- **Rigid body dynamics**: Motion of solid objects under applied forces
- **Collision detection**: Identifying when objects make contact
- **Contact response**: Calculating forces when objects touch
- **Friction modeling**: Simulating surface interactions and grip

### Multi-Body Dynamics
For humanoid robots with multiple interconnected links:
- **Forward dynamics**: Computing motion from applied forces
- **Inverse dynamics**: Computing forces needed for desired motion
- **Constraint solving**: Maintaining joint relationships
- **Stability**: Maintaining numerical stability in complex systems

### Sensor Simulation Theory
Accurate sensor simulation requires modeling:
- **Physical principles**: How sensors actually work (optics, magnetism, etc.)
- **Noise characteristics**: Realistic sensor noise and uncertainty
- **Latency**: Communication and processing delays
- **Bandwidth limitations**: Data rate constraints

### Realism vs. Performance Trade-offs
Simulation design involves balancing:
- Physical accuracy vs. computational performance
- Sensor realism vs. simulation speed
- Environmental complexity vs. stability
- Model detail vs. real-time constraints

## Implementation

Let's implement realistic physics and sensor configurations for humanoid robotics in Gazebo:

### Advanced URDF with Physics Properties

```xml
<?xml version="1.0" ?>
<robot name="advanced_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.423529411765 0.0392156862745 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base link with detailed physics -->
  <link name="base_link">
    <inertial>
      <mass value="15.0"/>
      <origin xyz="0 0 0.15"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.3" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <mesh filename="package://my_humanoid_description/meshes/base_link.dae"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.15"/>
      <geometry>
        <box size="0.3 0.25 0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso with COM adjustment -->
  <link name="torso">
    <inertial>
      <mass value="8.0"/>
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.15" iyz="0.0" izz="0.08"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <mesh filename="package://my_humanoid_description/meshes/torso.dae"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.2 0.15 0.5"/>
      </geometry>
    </collision>
  </link>

  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <!-- Head with realistic properties -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0.08"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.08"/>
      <geometry>
        <mesh filename="package://my_humanoid_description/meshes/head.dae"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.08"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="15.0" velocity="3.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- Left leg with detailed physics -->
  <link name="left_hip">
    <inertial>
      <mass value="3.0"/>
      <origin xyz="0 0 -0.1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <mesh filename="package://my_humanoid_description/meshes/hip.dae"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="-0.08 0.0 0.15"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="5.0"/>
    <dynamics damping="1.0" friction="0.5"/>
  </joint>

  <!-- Left knee -->
  <link name="left_knee">
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.004"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <mesh filename="package://my_humanoid_description/meshes/knee.dae"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_hip"/>
    <child link="left_knee"/>
    <origin xyz="0 0 -0.2"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.35" effort="100.0" velocity="5.0"/>
    <dynamics damping="1.0" friction="0.5"/>
  </joint>

  <!-- Left ankle -->
  <link name="left_ankle">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.05"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.002"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05"/>
      <geometry>
        <mesh filename="package://my_humanoid_description/meshes/ankle.dae"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.05"/>
      <geometry>
        <box size="0.1 0.08 0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_knee"/>
    <child link="left_ankle"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="50.0" velocity="3.0"/>
    <dynamics damping="0.5" friction="0.2"/>
  </joint>

  <!-- Left foot -->
  <link name="left_foot">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.05 0 -0.02"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02"/>
      <geometry>
        <mesh filename="package://my_humanoid_description/meshes/foot.dae"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.02"/>
      <geometry>
        <box size="0.2 0.1 0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_foot_joint" type="fixed">
    <parent link="left_ankle"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.1"/>
  </joint>

  <!-- Gazebo plugins for physics and sensors -->
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find my_humanoid_description)/config/humanoid_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Gazebo material properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <fdir1>0 0 1</fdir1>
    <maxVel>100.0</maxVel>
    <minDepth>0.001</minDepth>
  </gazebo>

  <gazebo reference="torso">
    <material>Gazebo/Blue</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/White</material>
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <!-- Left leg physics properties -->
  <gazebo reference="left_hip">
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo reference="left_knee">
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo reference="left_ankle">
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
  </gazebo>

  <gazebo reference="left_foot">
    <mu1>0.8</mu1>
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <fdir1>1 0 0</fdir1> <!-- Friction direction for foot contact -->
  </gazebo>

  <!-- Camera sensor with realistic parameters -->
  <gazebo reference="head">
    <sensor name="head_camera" type="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <camera name="head_camera">
        <horizontal_fov>1.047</horizontal_fov> <!-- 60 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>head_camera_optical_frame</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>10.0</max_depth>
        <update_rate>30.0</update_rate>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU sensor with realistic noise -->
  <gazebo reference="torso">
    <sensor name="torso_imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0017</stddev> <!-- ~0.1 deg/s -->
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0017</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.0017</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev> <!-- ~0.017 m/s^2 -->
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
        <frame_name>torso_imu_frame</frame_name>
        <body_name>torso</body_name>
        <topic>__default_topic__</topic>
        <serviceName>__default_service_name__</serviceName>
        <gaussianNoise>0.01</gaussianNoise>
        <updateRateHZ>100.0</updateRateHZ>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Force/Torque sensor for foot contact -->
  <gazebo reference="left_foot">
    <sensor name="left_foot_ft_sensor" type="force_torque">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <force_torque>
        <frame>child</frame>
        <measure_direction>child_to_parent</measure_direction>
      </force_torque>
      <plugin name="left_foot_ft_plugin" filename="libgazebo_ros_ft_sensor.so">
        <frame_name>left_foot</frame_name>
        <topic>left_foot/ft_sensor</topic>
      </plugin>
    </sensor>
  </gazebo>

  <!-- LiDAR sensor for environment perception -->
  <gazebo reference="head">
    <sensor name="head_lidar" type="ray">
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle> <!-- -90 degrees -->
            <max_angle>1.570796</max_angle>   <!-- 90 degrees -->
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="head_lidar_controller" filename="libgazebo_ros_laser.so">
        <frame_name>head_lidar_frame</frame_name>
        <topic>head_lidar/scan</topic>
        <gaussianNoise>0.01</gaussianNoise>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

### Physics Configuration File

```xml
<!-- config/physics_config.xml -->
<gazebo>
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
    <real_time_update_rate>1000.0</real_time_update_rate>
    <gravity>0 0 -9.8</gravity>
    <ode>
      <solver>
        <type>quick</type>
        <iters>10</iters>
        <sor>1.3</sor>
      </solver>
      <constraints>
        <cfm>0.0</cfm>
        <erp>0.2</erp>
        <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>
</gazebo>
```

### Sensor Processing Node

```python
#!/usr/bin/env python3
# scripts/sensor_processing_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time
import numpy as np
import cv2
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
import tf_transformations


class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Declare parameters
        self.declare_parameter('processing_frequency', 100.0)
        self.declare_parameter('image_processing_enabled', True)
        self.declare_parameter('imu_calibration_enabled', True)

        # Get parameters
        self.processing_frequency = self.get_parameter('processing_frequency').value
        self.image_processing_enabled = self.get_parameter('image_processing_enabled').value
        self.imu_calibration_enabled = self.get_parameter('imu_calibration_enabled').value

        # Publishers
        self.processed_sensor_pub = self.create_publisher(
            Float64MultiArray,
            'processed_sensors',
            10
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        if self.image_processing_enabled:
            self.image_sub = self.create_subscription(
                Image,
                'camera/image_raw',
                self.image_callback,
                10
            )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )

        # Timer for processing loop
        self.processing_timer = self.create_timer(
            1.0/self.processing_frequency,
            self.processing_loop
        )

        # Internal state
        self.current_imu = Imu()
        self.current_joint_states = JointState()
        self.current_lidar = LaserScan()
        self.cv_bridge = CvBridge()
        self.last_image_time = Time()
        self.balance_estimator = BalanceEstimator()

        self.get_logger().info('Sensor Processor initialized')

    def imu_callback(self, msg):
        """Process IMU data"""
        self.current_imu = msg

        # Apply calibration if enabled
        if self.imu_calibration_enabled:
            self.current_imu = self.calibrate_imu(msg)

    def joint_state_callback(self, msg):
        """Process joint state data"""
        self.current_joint_states = msg

    def image_callback(self, msg):
        """Process camera image data"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform basic image processing
            processed_data = self.process_image(cv_image)

            self.get_logger().debug(f'Processed image: {processed_data}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def lidar_callback(self, msg):
        """Process LiDAR scan data"""
        self.current_lidar = msg

        # Process scan for obstacles
        obstacles = self.detect_obstacles(msg)
        self.get_logger().debug(f'Detected {len(obstacles)} obstacles')

    def processing_loop(self):
        """Main processing loop"""
        # Estimate balance state from sensor data
        balance_state = self.balance_estimator.estimate_balance(
            self.current_imu,
            self.current_joint_states
        )

        # Process sensor fusion
        fused_data = self.fuse_sensors(
            self.current_imu,
            self.current_joint_states,
            self.current_lidar
        )

        # Publish processed data
        processed_msg = Float64MultiArray()
        processed_msg.data = fused_data
        self.processed_sensor_pub.publish(processed_msg)

        self.get_logger().debug(f'Published processed sensor data: {len(fused_data)} values')

    def calibrate_imu(self, raw_imu):
        """Apply IMU calibration"""
        calibrated_imu = Imu()
        calibrated_imu.header = raw_imu.header

        # Apply bias corrections (these would be calibrated values)
        bias_offset = Vector3(x=0.01, y=-0.02, z=0.005)

        calibrated_imu.linear_acceleration.x = raw_imu.linear_acceleration.x + bias_offset.x
        calibrated_imu.linear_acceleration.y = raw_imu.linear_acceleration.y + bias_offset.y
        calibrated_imu.linear_acceleration.z = raw_imu.linear_acceleration.z + bias_offset.z

        calibrated_imu.angular_velocity.x = raw_imu.angular_velocity.x
        calibrated_imu.angular_velocity.y = raw_imu.angular_velocity.y
        calibrated_imu.angular_velocity.z = raw_imu.angular_velocity.z

        calibrated_imu.orientation = raw_imu.orientation

        return calibrated_imu

    def process_image(self, cv_image):
        """Perform basic image processing"""
        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Perform edge detection
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Return count of contours as simple processed data
        return [len(contours)]

    def detect_obstacles(self, scan_msg):
        """Detect obstacles from LiDAR scan"""
        obstacles = []
        for i, range_val in enumerate(scan_msg.ranges):
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                # Check if obstacle is close enough to be significant
                if range_val < 1.0:  # 1 meter threshold
                    obstacles.append((angle, range_val))
        return obstacles

    def fuse_sensors(self, imu_data, joint_data, lidar_data):
        """Simple sensor fusion"""
        fused_data = []

        # IMU data
        fused_data.append(imu_data.linear_acceleration.x)
        fused_data.append(imu_data.linear_acceleration.y)
        fused_data.append(imu_data.linear_acceleration.z)

        # Joint positions
        for pos in joint_data.position:
            fused_data.append(pos)

        # LiDAR data (first few ranges as example)
        if len(lidar_data.ranges) > 0:
            for i in range(min(10, len(lidar_data.ranges))):
                fused_data.append(lidar_data.ranges[i])

        return fused_data


class BalanceEstimator:
    """Estimate robot balance state from sensor data"""

    def __init__(self):
        self.com_estimator = CenterOfMassEstimator()
        self.stability_threshold = 0.1  # meters

    def estimate_balance(self, imu_data, joint_data):
        """Estimate balance based on IMU and joint data"""
        # Calculate center of mass position
        com_pos = self.com_estimator.calculate_com(joint_data.position)

        # Get orientation from IMU
        orientation = imu_data.orientation
        euler = tf_transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        # Calculate stability metrics
        roll, pitch, yaw = euler
        stability = np.sqrt(roll**2 + pitch**2)

        return {
            'com_x': com_pos[0],
            'com_y': com_pos[1],
            'com_z': com_pos[2],
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw,
            'stability': stability,
            'is_stable': stability < self.stability_threshold
        }


class CenterOfMassEstimator:
    """Calculate center of mass based on joint positions"""

    def __init__(self):
        # Mass properties for each link (simplified)
        self.link_masses = {
            'base_link': 15.0,
            'torso': 8.0,
            'head': 2.0,
            'left_hip': 3.0,
            'left_knee': 2.5,
            'left_ankle': 1.5,
            'left_foot': 1.0
        }

    def calculate_com(self, joint_positions):
        """Calculate center of mass (simplified)"""
        # This is a simplified calculation - in reality, you'd need
        # the full kinematic chain and link positions
        total_mass = sum(self.link_masses.values())

        # For demonstration, return a simple calculation
        # In practice, you'd use forward kinematics
        com_x = 0.0
        com_y = 0.0
        com_z = 0.8  # Approximate CoM height for humanoid

        return [com_x, com_y, com_z]


def main(args=None):
    rclpy.init(args=args)

    processor = SensorProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        processor.get_logger().info('Shutting down sensor processor...')
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Hardware/GPU Notes

Physics simulation and sensor modeling for humanoid robotics have specific hardware requirements:

### Physics Simulation Requirements
- **CPU**: Multi-core processor (8+ cores recommended) for real-time physics
- **Memory**: 16GB+ for complex humanoid models with multiple contacts
- **Physics Engine**: ODE, Bullet, or DART with appropriate tuning
- **Real-time Kernel**: Consider PREEMPT_RT for deterministic physics

### Sensor Simulation Requirements
- **GPU**: Modern graphics card with CUDA support for camera simulation
- **VRAM**: 8GB+ for detailed camera and LiDAR simulation
- **Memory Bandwidth**: High bandwidth for sensor data processing
- **Processing Power**: Real-time sensor data generation and processing

### Performance Optimization
- Use simplified collision meshes for physics (keep detailed visuals separate)
- Limit sensor update rates to realistic values
- Optimize contact parameters for stability
- Use appropriate solver parameters for humanoid dynamics

## Simulation Path

For developing realistic physics and sensor simulation for humanoid robotics:

### Physics Setup
1. Configure physics engine parameters for humanoid dynamics
2. Set realistic mass, inertia, and friction properties
3. Optimize solver parameters for stability
4. Test single-joint physics before complex models

### Sensor Configuration
1. Model real sensor characteristics and noise
2. Configure realistic update rates and ranges
3. Validate sensor data against hardware specifications
4. Test sensor fusion algorithms in simulation

### Validation Process
1. Test physics stability with complex humanoid models
2. Validate sensor data realism and noise characteristics
3. Compare simulation vs. real-world sensor data
4. Adjust parameters for better fidelity

## Real-World Path

Transitioning from simulation to real hardware:

### Physics Validation
1. Compare simulation and real-world dynamics
2. Validate contact forces and friction models
3. Adjust parameters based on real-world behavior
4. Document differences for compensation strategies

### Sensor Calibration
1. Calibrate simulated sensors to match hardware
2. Validate noise models against real sensors
3. Adjust simulation parameters for better match
4. Test sensor fusion algorithms with both datasets

### Safety Considerations
1. Implement safety limits based on simulation results
2. Validate emergency stop procedures with physics
3. Test sensor failure scenarios
4. Ensure safe operation boundaries are respected

## Spec-Build-Test Checklist

- [ ] Physics parameters are realistic for humanoid dynamics
- [ ] Mass and inertia properties match real hardware
- [ ] Friction and contact parameters are properly configured
- [ ] Sensor noise models match real hardware characteristics
- [ ] Update rates are realistic for each sensor type
- [ ] Physics simulation runs at real-time speed
- [ ] Collision detection works properly for all links
- [ ] Sensor data is published at expected rates
- [ ] Joint limits and constraints are properly enforced
- [ ] Solver parameters provide stable simulation
- [ ] Multi-body dynamics behave realistically
- [ ] Sensor fusion algorithms work with simulated data
- [ ] Emergency stop procedures work with physics simulation
- [ ] Performance metrics are monitored during simulation

## APA Citations

- Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 2350-2354.
- Tedrake, R., Jackowski, Z., Miller, R., Murphey, J., & Erez, T. (2010). Using system identification to obtain reliable models for legged robots. *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 1417-1423.
- Coumans, E., & Bai, Y. (2016). Mujoco: A physics engine for model-based control. *IEEE International Conference on Robotics and Automation*.
- Murai, R., & Kyrki, V. (2021). Simulation to reality transfer in robotics: A survey. *IEEE Access*, 9, 142395-142418.
- Sadeghi, F., & Levine, S. (2017). CAD2RL: Real single-image flight without a single real image. *Proceedings of the International Conference on Robotics and Automation*.
- Xie, W., Tan, J., & Turk, G. (2020). Learning dexterous manipulation from random grasps. *IEEE Robotics and Automation Letters*, 5(2), 2810-2817.