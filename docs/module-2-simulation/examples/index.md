---
title: Module 2 Example Code - Simulation Environments
sidebar_position: 16
---

# Module 2 Example Code: Simulation Environments

## 1. Basic Gazebo Robot Spawn Example

### URDF Model Definition

```xml
<?xml version="1.0" ?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>

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
      <material name="grey"/>
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
      <origin xyz="0 0 0.25"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.25"/>
      <geometry>
        <box size="0.2 0.15 0.5"/>
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
    <origin xyz="0 0 0.2"/>
  </joint>

  <!-- Head -->
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
      <material name="green"/>
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
    <origin xyz="0 0 0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10.0" velocity="2.0"/>
  </joint>

  <!-- Left leg -->
  <link name="left_hip">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="-0.08 0 0.1"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="3.0"/>
  </joint>

  <link name="left_knee">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.004"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_hip"/>
    <child link="left_knee"/>
    <origin xyz="0 0 -0.2"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.35" effort="50.0" velocity="3.0"/>
  </joint>

  <link name="left_ankle">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.05"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.002"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05"/>
      <geometry>
        <box size="0.1 0.08 0.1"/>
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
    <limit lower="-0.5" upper="0.5" effort="25.0" velocity="2.0"/>
  </joint>

  <!-- Right leg (mirror of left) -->
  <link name="right_hip">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.005"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1"/>
      <geometry>
        <cylinder length="0.2" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="0.08 0 0.1"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="3.0"/>
  </joint>

  <link name="right_knee">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.004"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_hip"/>
    <child link="right_knee"/>
    <origin xyz="0 0 -0.2"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="2.35" effort="50.0" velocity="3.0"/>
  </joint>

  <link name="right_ankle">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.05"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.002"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.05"/>
      <geometry>
        <box size="0.1 0.08 0.1"/>
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

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_knee"/>
    <child link="right_ankle"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" effort="25.0" velocity="2.0"/>
  </joint>

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find my_humanoid_description)/config/humanoid_controllers.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Gazebo materials -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="torso">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/Green</material>
  </gazebo>

  <gazebo reference="left_hip">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_knee">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="left_ankle">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_hip">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_knee">
    <material>Gazebo/Red</material>
  </gazebo>

  <gazebo reference="right_ankle">
    <material>Gazebo/Red</material>
  </gazebo>

</robot>
```

### Gazebo Launch File

```xml
<!-- launch/humanoid_spawn.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_description_path = LaunchConfiguration('robot_description_path')

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
            'simple_humanoid.urdf.xacro'
        ]),
        description='Path to robot URDF file'
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

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
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
            '-entity', 'simple_humanoid',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_robot_description_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
```

## 2. Unity-ROS Connection Example

### Unity Robot Controller Script

```csharp
// Assets/Scripts/UnityRobotController.cs
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry_msgs;
using System.Collections;
using System.Collections.Generic;

public class UnityRobotController : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    [Header("Robot Configuration")]
    public string jointStateTopic = "/joint_states";
    public string commandTopic = "/joint_trajectory_controller/joint_trajectory";
    public string imuTopic = "/imu/data";

    [Header("Joint Transforms")]
    public Transform headJoint;
    public Transform leftHipJoint;
    public Transform leftKneeJoint;
    public Transform leftAnkleJoint;
    public Transform rightHipJoint;
    public Transform rightKneeJoint;
    public Transform rightAnkleJoint;

    private ROSConnection ros;
    private Dictionary<string, Transform> jointMap;
    private JointStateData currentJointState;

    [System.Serializable]
    public class JointStateData
    {
        public string[] names;
        public double[] positions;
        public double[] velocities;
        public double[] efforts;
    }

    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.Initialize(rosIPAddress, rosPort);

        // Subscribe to topics
        ros.Subscribe<sensor_msgs_JointState>(jointStateTopic, OnJointStateReceived);
        ros.Subscribe<sensor_msgs_Imu>(imuTopic, OnImuReceived);

        // Initialize joint mapping
        InitializeJointMap();
    }

    void InitializeJointMap()
    {
        jointMap = new Dictionary<string, Transform>
        {
            {"head_joint", headJoint},
            {"left_hip_joint", leftHipJoint},
            {"left_knee_joint", leftKneeJoint},
            {"left_ankle_joint", leftAnkleJoint},
            {"right_hip_joint", rightHipJoint},
            {"right_knee_joint", rightKneeJoint},
            {"right_ankle_joint", rightAnkleJoint}
        };
    }

    void OnJointStateReceived(sensor_msgs_JointState jointState)
    {
        currentJointState = new JointStateData
        {
            names = jointState.name,
            positions = jointState.position,
            velocities = jointState.velocity,
            efforts = jointState.effort
        };

        // Update robot visualization
        UpdateRobotVisualization();
    }

    void OnImuReceived(sensor_msgs_Imu imu)
    {
        // Update robot orientation based on IMU data
        Vector3 eulerAngles = RosQuaternionToUnityEuler(imu.orientation);
        transform.rotation = Quaternion.Euler(eulerAngles);
    }

    void UpdateRobotVisualization()
    {
        if (currentJointState == null || currentJointState.names == null) return;

        for (int i = 0; i < currentJointState.names.Length; i++)
        {
            string jointName = currentJointState.names[i];
            float jointPosition = (float)currentJointState.positions[i];

            if (jointMap.ContainsKey(jointName))
            {
                Transform jointTransform = jointMap[jointName];
                // Apply rotation based on joint position (assuming revolute joints)
                jointTransform.localRotation = Quaternion.Euler(0, jointPosition * Mathf.Rad2Deg, 0);
            }
        }
    }

    Vector3 RosQuaternionToUnityEuler(Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry_msgs.Quaternion rosQuat)
    {
        float x = (float)rosQuat.x;
        float y = (float)rosQuat.y;
        float z = (float)rosQuat.z;
        float w = (float)rosQuat.w;

        // Convert to Euler angles
        Vector3 eulerAngles = new Vector3();

        // Yaw (Z axis rotation)
        eulerAngles.y = Mathf.Atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * Mathf.Rad2Deg;

        // Pitch (Y axis rotation)
        eulerAngles.x = Mathf.Atan2(2 * (w * y - z * x), Mathf.Sqrt(1 + 2 * (w * y - z * x) * (w * y - z * x))) * Mathf.Rad2Deg;

        // Roll (X axis rotation)
        eulerAngles.z = Mathf.Asin(2 * (w * x + y * z)) * Mathf.Rad2Deg;

        return eulerAngles;
    }

    public void SendJointCommand(string jointName, float position)
    {
        // This would send a command to the robot
        // Implementation depends on your specific command structure
    }
}
```

### Unity Visualization Script

```csharp
// Assets/Scripts/UnityRobotVisualizer.cs
using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class UnityRobotVisualizer : MonoBehaviour
{
    [Header("Visualization Settings")]
    public bool showTrajectory = true;
    public bool showSensors = true;
    public bool showForces = true;

    [Header("UI Elements")]
    public Text statusText;
    public Text jointInfoText;
    public Slider timeScaleSlider;

    [Header("Visualization Prefabs")]
    public GameObject trajectoryPointPrefab;
    public GameObject sensorRangeVisual;

    private LineRenderer trajectoryLine;
    private List<Vector3> trajectoryPoints;
    private UnityRobotController robotController;

    void Start()
    {
        // Initialize trajectory visualization
        InitializeTrajectory();

        // Get references
        robotController = FindObjectOfType<UnityRobotController>();

        // Setup UI
        SetupUI();
    }

    void InitializeTrajectory()
    {
        trajectoryLine = gameObject.AddComponent<LineRenderer>();
        trajectoryLine.material = new Material(Shader.Find("Sprites/Default"));
        trajectoryLine.widthMultiplier = 0.05f;
        trajectoryLine.positionCount = 0;

        trajectoryPoints = new List<Vector3>();
    }

    void SetupUI()
    {
        if (timeScaleSlider != null)
        {
            timeScaleSlider.onValueChanged.AddListener(OnTimeScaleChanged);
            timeScaleSlider.value = 1.0f;
        }
    }

    void OnTimeScaleChanged(float value)
    {
        Time.timeScale = value;
    }

    void Update()
    {
        // Update trajectory visualization
        UpdateTrajectory();

        // Update UI information
        UpdateUIInformation();
    }

    void UpdateTrajectory()
    {
        if (!showTrajectory) return;

        // Add current position to trajectory
        Vector3 currentPosition = transform.position;

        // Remove old points based on time
        float currentTime = Time.time;
        trajectoryPoints.Add(currentPosition);

        // Limit number of points to prevent memory issues
        if (trajectoryPoints.Count > 1000)
        {
            trajectoryPoints.RemoveAt(0);
        }

        // Update line renderer
        trajectoryLine.positionCount = trajectoryPoints.Count;
        trajectoryLine.SetPositions(trajectoryPoints.ToArray());
    }

    void UpdateUIInformation()
    {
        if (statusText != null)
        {
            statusText.text = $"Time Scale: {Time.timeScale:F2}x\n" +
                             $"Trajectory Points: {trajectoryPoints.Count}\n" +
                             $"Status: Connected";
        }

        if (jointInfoText != null && robotController != null)
        {
            var jointState = robotController.currentJointState;
            if (jointState != null && jointState.names != null)
            {
                string jointInfo = "Joint Positions:\n";
                for (int i = 0; i < Mathf.Min(5, jointState.names.Length); i++)
                {
                    jointInfo += $"{jointState.names[i]}: {jointState.positions[i]:F3}\n";
                }
                jointInfoText.text = jointInfo;
            }
        }
    }

    public void ToggleTrajectory()
    {
        showTrajectory = !showTrajectory;
        trajectoryLine.enabled = showTrajectory;
    }

    public void ClearTrajectory()
    {
        trajectoryPoints.Clear();
        trajectoryLine.positionCount = 0;
    }
}
```

## 3. Digital Twin Core Example

### Digital Twin Core Node

```python
#!/usr/bin/env python3
# digital_twin_core_example.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Pose, Twist
import numpy as np
import threading
from collections import deque
from dataclasses import dataclass
from typing import List


@dataclass
class RobotState:
    """Data structure for robot state in digital twin"""
    joint_positions: List[float]
    joint_velocities: List[float]
    joint_efforts: List[float]
    imu_orientation: List[float]  # x, y, z, w quaternion
    imu_angular_velocity: List[float]  # x, y, z
    imu_linear_acceleration: List[float]  # x, y, z
    pose: Pose
    velocity: Twist


class DigitalTwinCore(Node):
    def __init__(self):
        super().__init__('digital_twin_core')

        # Publishers
        self.twin_state_pub = self.create_publisher(
            Float64MultiArray,
            'digital_twin/state',
            10
        )

        # Subscribers
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

        # Timer for twin update
        self.twin_timer = self.create_timer(0.01, self.twin_update_loop)  # 100 Hz

        # Internal state
        self.current_state = RobotState(
            joint_positions=[],
            joint_velocities=[],
            joint_efforts=[],
            imu_orientation=[0.0, 0.0, 0.0, 1.0],
            imu_angular_velocity=[0.0, 0.0, 0.0],
            imu_linear_acceleration=[0.0, 0.0, 0.0],
            pose=Pose(),
            velocity=Twist()
        )

        self.state_history = deque(maxlen=1000)
        self.state_lock = threading.RLock()

        self.get_logger().info('Digital Twin Core initialized')

    def joint_state_callback(self, msg):
        """Update joint state in digital twin"""
        with self.state_lock:
            self.current_state.joint_positions = list(msg.position)
            self.current_state.joint_velocities = list(msg.velocity)
            self.current_state.joint_efforts = list(msg.effort)

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

    def twin_update_loop(self):
        """Main digital twin update loop"""
        with self.state_lock:
            # Add current state to history
            self.state_history.append(self.current_state)

            # Publish current twin state
            state_msg = Float64MultiArray()
            state_msg.data = self.pack_state_for_publishing(self.current_state)
            self.twin_state_pub.publish(state_msg)

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
        return data


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

## 4. Simulation Configuration Manager Example

### Configuration Manager Node

```python
#!/usr/bin/env python3
# simulation_config_manager_example.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import json
import os
from dataclasses import dataclass, asdict
from typing import List


@dataclass
class PhysicsParameters:
    """Physics engine parameters for humanoid simulation"""
    solver_type: str = "quick"
    solver_iterations: int = 100
    solver_sor: float = 1.3
    max_step_size: float = 0.001
    real_time_factor: float = 1.0
    gravity: List[float] = None

    def __post_init__(self):
        if self.gravity is None:
            self.gravity = [0.0, 0.0, -9.81]


class SimulationConfigManager(Node):
    def __init__(self):
        super().__init__('simulation_config_manager')

        # Publishers
        self.config_status_pub = self.create_publisher(
            String,
            'simulation_config/status',
            10
        )

        # Internal state
        self.physics_params = PhysicsParameters()

        # Load initial configuration
        self.load_configuration()

        self.get_logger().info('Simulation Configuration Manager initialized')

    def load_configuration(self):
        """Load simulation configuration from file"""
        config_file = 'config/simulation_config.yaml'
        try:
            if os.path.exists(config_file):
                with open(config_file, 'r') as f:
                    config_data = yaml.safe_load(f)

                if 'physics' in config_data:
                    physics_data = config_data['physics']
                    self.physics_params = PhysicsParameters(
                        solver_type=physics_data.get('solver_type', 'quick'),
                        solver_iterations=physics_data.get('solver_iterations', 100),
                        solver_sor=physics_data.get('solver_sor', 1.3),
                        max_step_size=physics_data.get('max_step_size', 0.001),
                        real_time_factor=physics_data.get('real_time_factor', 1.0),
                        gravity=physics_data.get('gravity', [0.0, 0.0, -9.81])
                    )

                self.get_logger().info(f'Configuration loaded from {config_file}')
            else:
                self.get_logger().warn(f'Config file {config_file} not found, using defaults')

        except Exception as e:
            self.get_logger().error(f'Error loading configuration: {str(e)}')

    def save_configuration(self, file_path='config/simulation_config.yaml'):
        """Save current configuration to file"""
        config_data = {
            'physics': asdict(self.physics_params)
        }

        try:
            with open(file_path, 'w') as f:
                yaml.dump(config_data, f, default_flow_style=False)
            self.get_logger().info(f'Configuration saved to {file_path}')
        except Exception as e:
            self.get_logger().error(f'Error saving configuration: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    config_manager = SimulationConfigManager()

    try:
        rclpy.spin(config_manager)
    except KeyboardInterrupt:
        config_manager.get_logger().info('Shutting down simulation config manager...')
        config_manager.save_configuration()
    finally:
        config_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 5. Transfer Validation System Example

### Transfer Validation Node

```python
#!/usr/bin/env python3
# transfer_validation_example.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState
import numpy as np
from dataclasses import dataclass


@dataclass
class TransferMetrics:
    """Metrics for simulation to reality transfer"""
    position_error: float = 0.0
    velocity_error: float = 0.0
    orientation_error: float = 0.0
    transfer_score: float = 0.0


class TransferValidationSystem(Node):
    def __init__(self):
        super().__init__('transfer_validation_system')

        # Publishers
        self.transfer_metrics_pub = self.create_publisher(
            Float64MultiArray,
            'transfer_validation/metrics',
            10
        )

        self.transfer_status_pub = self.create_publisher(
            Bool,
            'transfer_validation/status',
            10
        )

        # Subscribers for sim and real robot data
        self.sim_joint_state_sub = self.create_subscription(
            JointState,
            '/sim/joint_states',
            self.sim_joint_state_callback,
            10
        )

        self.real_joint_state_sub = self.create_subscription(
            JointState,
            '/real/joint_states',
            self.real_joint_state_callback,
            10
        )

        # Timer for transfer validation
        self.validation_timer = self.create_timer(0.1, self.transfer_validation_loop)

        # Internal state
        self.sim_joint_state = None
        self.real_joint_state = None
        self.current_metrics = TransferMetrics()

        self.get_logger().info('Transfer Validation System initialized')

    def sim_joint_state_callback(self, msg):
        """Process simulation joint state data"""
        self.sim_joint_state = msg

    def real_joint_state_callback(self, msg):
        """Process real robot joint state data"""
        self.real_joint_state = msg

    def transfer_validation_loop(self):
        """Main transfer validation loop"""
        # Calculate transfer metrics
        self.current_metrics = self.calculate_transfer_metrics()

        # Publish metrics
        metrics_msg = Float64MultiArray()
        metrics_msg.data = [
            self.current_metrics.position_error,
            self.current_metrics.velocity_error,
            self.current_metrics.orientation_error,
            self.current_metrics.transfer_score
        ]
        self.transfer_metrics_pub.publish(metrics_msg)

        # Publish status
        status_msg = Bool()
        status_msg.data = self.current_metrics.transfer_score >= 0.8  # 80% threshold
        self.transfer_status_pub.publish(status_msg)

    def calculate_transfer_metrics(self) -> TransferMetrics:
        """Calculate comprehensive transfer metrics"""
        metrics = TransferMetrics()

        if self.sim_joint_state is None or self.real_joint_state is None:
            return metrics

        # Calculate position error (if joint names match)
        if (self.sim_joint_state.name == self.real_joint_state.name and
            len(self.sim_joint_state.position) == len(self.real_joint_state.position)):
            pos_errors = []
            for sim_pos, real_pos in zip(self.sim_joint_state.position, self.real_joint_state.position):
                pos_errors.append(abs(sim_pos - real_pos))
            metrics.position_error = np.mean(pos_errors) if pos_errors else 0.0

        # Calculate velocity error
        if (len(self.sim_joint_state.velocity) == len(self.real_joint_state.velocity) and
            len(self.sim_joint_state.velocity) > 0):
            vel_errors = []
            for sim_vel, real_vel in zip(self.sim_joint_state.velocity, self.real_joint_state.velocity):
                vel_errors.append(abs(sim_vel - real_vel))
            metrics.velocity_error = np.mean(vel_errors) if vel_errors else 0.0

        # Calculate transfer score (simplified)
        position_score = max(0.0, 1.0 - metrics.position_error / 0.1)  # 0.1m tolerance
        velocity_score = max(0.0, 1.0 - metrics.velocity_error / 1.0)  # 1.0 rad/s tolerance

        metrics.transfer_score = np.mean([position_score, velocity_score])

        return metrics


def main(args=None):
    rclpy.init(args=args)

    transfer_system = TransferValidationSystem()

    try:
        rclpy.spin(transfer_system)
    except KeyboardInterrupt:
        transfer_system.get_logger().info('Shutting down transfer validation system...')
    finally:
        transfer_system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## 6. Simulation World File Example

### Gazebo World with Humanoid Environment

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_test_world">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sky -->
    <include>
      <uri>model://sky</uri>
    </include>

    <!-- Physics engine configuration -->
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

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.3 -1</direction>
    </light>

    <!-- Simple obstacles for humanoid testing -->
    <model name="obstacle_1">
      <pose>2 0 0.2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.4</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <model name="obstacle_2">
      <pose>-2 1 0.1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.2</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.8 1</ambient>
            <diffuse>0.5 0.5 1.0 1</diffuse>
            <specular>0.1 0.1 0.3 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Ramps for testing locomotion -->
    <model name="ramp_1">
      <pose>3 -2 0 0 0 0.5</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2.0 1.0 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2.0 1.0 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.6 0.2 1</ambient>
            <diffuse>1.0 0.8 0.4 1</diffuse>
            <specular>0.3 0.2 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

## 7. Controller Configuration Example

### ROS 2 Controller Configuration

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

## APA Citations

- Koenig, N., & Howard, A. (2004). Design and use paradigms for Gazebo, an open-source multi-robot simulator. *IEEE/RSJ International Conference on Intelligent Robots and Systems*, 2350-2354.
- Unity Technologies. (2022). Unity Robotics Hub: Documentation and tutorials. *Unity Developer Documentation*.
- Open Robotics. (2022). ROS 2 and Unity integration: Best practices for robotics simulation. *ROS 2 Developer Guide*.
- Murai, R., & Kyrki, V. (2021). Simulation to reality transfer in robotics: A survey. *IEEE Access*, 9, 142395-142418.
- Sadeghi, F., & Levine, S. (2017). CAD2RL: Real single-image flight without a single real image. *Proceedings of the International Conference on Robotics and Automation*.
- Coumans, E., & Bai, Y. (2016). Mujoco: A physics engine for model-based control. *IEEE International Conference on Robotics and Automation*.