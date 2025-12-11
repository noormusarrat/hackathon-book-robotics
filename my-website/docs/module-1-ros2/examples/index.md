---
title: Module 1 Example Code
sidebar_position: 2
---

# Module 1 Example Code: ROS 2 Fundamentals

This section contains complete, runnable example code that demonstrates the core concepts covered in Module 1. Each example is designed to be educational and practical for humanoid robotics applications.

## 1. Basic Publisher-Subscriber Example

### Publisher Node

```python
#!/usr/bin/env python3
# publisher_example.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Header
import time

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 2)
        self.declare_parameter('message_content', 'Hello, ROS 2!')

        # Get parameter values
        self.publish_rate = self.get_parameter('publish_rate').value
        self.message_content = self.get_parameter('message_content').value

        # Create publisher
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Create timer for publishing
        self.timer = self.create_timer(1.0/self.publish_rate, self.timer_callback)

        # Counter for messages
        self.i = 0

        self.get_logger().info(
            f'Publisher initialized with rate {self.publish_rate}Hz and message: "{self.message_content}"'
        )

    def timer_callback(self):
        msg = String()
        msg.data = f'{self.message_content} - {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        minimal_publisher.get_logger().info('Shutting down publisher...')
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Node

```python
#!/usr/bin/env python3
# subscriber_example.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('Subscriber initialized, listening to "topic"...')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        minimal_subscriber.get_logger().info('Shutting down subscriber...')
    finally:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2. Service Example

### Service Definition (add_multiply.srv)

```
# Request
float64 a
float64 b
---
# Response
float64 sum
float64 product
```

### Service Server

```python
#!/usr/bin/env python3
# service_server_example.py

import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AddMultiply  # Custom service

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')

        # Create service
        self.srv = self.create_service(
            AddMultiply,
            'add_multiply',
            self.add_multiply_callback
        )

        self.get_logger().info('Service server started, waiting for requests...')

    def add_multiply_callback(self, request, response):
        # Perform calculations
        response.sum = request.a + request.b
        response.product = request.a * request.b

        self.get_logger().info(
            f'Calculating: {request.a} + {request.b} = {response.sum}, '
            f'{request.a} * {request.b} = {response.product}'
        )

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    try:
        rclpy.spin(minimal_service)
    except KeyboardInterrupt:
        minimal_service.get_logger().info('Shutting down service...')
    finally:
        minimal_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client

```python
#!/usr/bin/env python3
# service_client_example.py

import sys
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import AddMultiply  # Custom service

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')

        # Create client
        self.cli = self.create_client(AddMultiply, 'add_multiply')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service client ready')

    def send_request(self, a, b):
        request = AddMultiply.Request()
        request.a = float(a)
        request.b = float(b)

        self.future = self.cli.call_async(request)
        return self.future

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClient()

    # Check if arguments are provided
    if len(sys.argv) != 3:
        print('Usage: python3 service_client_example.py <a> <b>')
        return

    future = minimal_client.send_request(sys.argv[1], sys.argv[2])

    try:
        rclpy.spin_until_future_complete(minimal_client, future)

        if future.result() is not None:
            response = future.result()
            print(f'Result: {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
            print(f'Result: {sys.argv[1]} * {sys.argv[2]} = {response.product}')
        else:
            minimal_client.get_logger().error('Service call failed')

    except KeyboardInterrupt:
        minimal_client.get_logger().info('Shutting down client...')
    finally:
        minimal_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. Action Example

### Action Definition (fibonacci.action)

```
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] sequence
```

### Action Server

```python
#!/usr/bin/env python3
# action_server_example.py

import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from my_robot_interfaces.action import Fibonacci  # Custom action

class MinimalActionServer(Node):
    def __init__(self):
        super().__init__('minimal_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Action server started, waiting for goals...')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action"""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal and provide feedback"""
        self.get_logger().info('Executing goal...')

        # Get the goal order
        order = goal_handle.request.order

        # Feedback and result
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0]
        result = Fibonacci.Result()

        # Calculate Fibonacci sequence
        if order == 0:
            result.sequence = [0]
        elif order == 1:
            result.sequence = [0, 1]
        else:
            sequence = [0, 1]
            feedback_msg.sequence = sequence[:]

            for i in range(1, order):
                if goal_handle.is_cancel_requested:
                    self.get_logger().info('Goal was canceled')
                    goal_handle.canceled()
                    result.sequence = sequence
                    return result

                # Update sequence
                next_fib = sequence[i] + sequence[i-1]
                sequence.append(next_fib)

                # Publish feedback
                feedback_msg.sequence = sequence[:]
                goal_handle.publish_feedback(feedback_msg)

                # Sleep to simulate work
                time.sleep(0.5)

            result.sequence = sequence

        # Check if goal was canceled
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled during execution')
        else:
            goal_handle.succeed()
            self.get_logger().info('Goal succeeded')

        return result

def main(args=None):
    rclpy.init(args=args)

    minimal_action_server = MinimalActionServer()

    try:
        rclpy.spin(minimal_action_server)
    except KeyboardInterrupt:
        minimal_action_server.get_logger().info('Shutting down action server...')
    finally:
        minimal_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Action Client

```python
#!/usr/bin/env python3
# action_client_example.py

import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_interfaces.action import Fibonacci  # Custom action

class MinimalActionClient(Node):
    def __init__(self):
        super().__init__('minimal_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

        self.get_logger().info('Action client created')

    def send_goal(self, order):
        # Wait for action server
        self._action_client.wait_for_server()

        # Create goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # Check if argument is provided
    if len(sys.argv) != 2:
        print('Usage: python3 action_client_example.py <order>')
        return

    action_client = MinimalActionClient()

    # Send goal with specified order
    action_client.send_goal(int(sys.argv[1]))

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        action_client.get_logger().info('Shutting down action client...')

if __name__ == '__main__':
    main()
```

## 4. Parameter Example

```python
#!/usr/bin/env python3
# parameter_example.py

import rclpy
from rclpy.node import Node

class ParameterExample(Node):
    def __init__(self):
        super().__init__('parameter_example')

        # Declare parameters with default values and descriptions
        self.declare_parameter('robot_name', 'my_robot',
                             rclpy.ParameterDescriptor(description='Name of the robot'))
        self.declare_parameter('max_velocity', 1.0,
                             rclpy.ParameterDescriptor(description='Maximum velocity (m/s)'))
        self.declare_parameter('wheel_diameter', 0.1,
                             rclpy.ParameterDescriptor(description='Wheel diameter (m)'))
        self.declare_parameter('joint_limits', [1.57, 1.57, 1.57],
                             rclpy.ParameterDescriptor(description='Joint limits in radians'))

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.joint_limits = self.get_parameter('joint_limits').value

        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'Robot name: {self.robot_name}')
        self.get_logger().info(f'Max velocity: {self.max_velocity} m/s')
        self.get_logger().info(f'Wheel diameter: {self.wheel_diameter} m')
        self.get_logger().info(f'Joint limits: {self.joint_limits}')

        # Create a timer to periodically check parameter values
        self.timer = self.create_timer(2.0, self.timer_callback)

    def parameter_callback(self, params):
        """Callback for parameter changes"""
        for param in params:
            self.get_logger().info(f'Parameter {param.name} changed to {param.value}')

            # Update internal values if needed
            if param.name == 'max_velocity':
                if param.value <= 0:
                    return rclpy.parameter.ParameterEvent()

        return rclpy.node.SetParametersResult(successful=True)

    def timer_callback(self):
        """Timer callback to check current parameter values"""
        current_max_velocity = self.get_parameter('max_velocity').value
        self.get_logger().info(f'Current max velocity: {current_max_velocity} m/s')

def main(args=None):
    rclpy.init(args=args)

    param_example = ParameterExample()

    try:
        rclpy.spin(param_example)
    except KeyboardInterrupt:
        param_example.get_logger().info('Shutting down parameter example...')
    finally:
        param_example.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Launch File Example

```python
#!/usr/bin/env python3
# multi_node_example.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_rate = LaunchConfiguration('publish_rate')
    robot_namespace = LaunchConfiguration('robot_namespace')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='2',
            description='Publish rate for the publisher node'
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='robot1',
            description='Robot namespace'
        ),

        # Publisher node
        Node(
            package='my_robot_tutorials',
            executable='publisher_example',
            name='publisher_node',
            namespace=robot_namespace,
            parameters=[
                {'publish_rate': publish_rate},
                {'message_content': 'Hello from launch file!'},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        # Subscriber node
        Node(
            package='my_robot_tutorials',
            executable='subscriber_example',
            name='subscriber_node',
            namespace=robot_namespace,
            parameters=[
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        # Parameter example node
        Node(
            package='my_robot_tutorials',
            executable='parameter_example',
            name='parameter_example_node',
            namespace=robot_namespace,
            parameters=[
                {'robot_name': 'launch_robot'},
                {'max_velocity': 2.0},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        )
    ])
```

## 6. Configuration File Example (YAML)

```yaml
# config/robot_config.yaml
/**:
  ros__parameters:
    robot_name: "humanoid_robot"
    max_velocity: 1.5
    wheel_diameter: 0.15
    joint_limits:
      - 1.57
      - 1.57
      - 1.57
      - 2.0
    safety:
      max_effort: 100.0
      emergency_stop_timeout: 0.1
    control:
      rate: 100
      kp: 10.0
      ki: 0.1
      kd: 0.05
    sensors:
      camera:
        resolution: [640, 480]
        fps: 30
      lidar:
        range_min: 0.1
        range_max: 10.0
        angle_min: -2.35619  # -135 degrees
        angle_max: 2.35619   # 135 degrees
```

## 7. Package.xml Example

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_tutorials</name>
  <version>1.0.0</version>
  <description>Example ROS 2 packages for humanoid robotics tutorials</description>
  <maintainer email="tutorials@robotics.org">Robotics Tutorials</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>action_msgs</depend>
  <depend>example_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## 8. Setup.py Example

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_tutorials'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Tutorials',
    maintainer_email='tutorials@robotics.org',
    description='Example ROS 2 packages for humanoid robotics tutorials',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_example = my_robot_tutorials.publisher_example:main',
            'subscriber_example = my_robot_tutorials.subscriber_example:main',
            'service_server_example = my_robot_tutorials.service_server_example:main',
            'service_client_example = my_robot_tutorials.service_client_example:main',
            'action_server_example = my_robot_tutorials.action_server_example:main',
            'action_client_example = my_robot_tutorials.action_client_example:main',
            'parameter_example = my_robot_tutorials.parameter_example:main',
        ],
    },
)
```

## Running the Examples

To run these examples:

1. Create a new ROS 2 package:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_robot_tutorials
   ```

2. Copy the example code into the appropriate files in your package

3. Update setup.py with the console scripts

4. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_tutorials
   source install/setup.bash
   ```

5. Run individual nodes:
   ```bash
   ros2 run my_robot_tutorials publisher_example
   ros2 run my_robot_tutorials subscriber_example
   ```

6. Or run with launch file:
   ```bash
   ros2 launch my_robot_tutorials multi_node_example.launch.py
   ```

These examples provide a comprehensive foundation for understanding ROS 2 concepts in the context of humanoid robotics applications.

## APA Citations

- Open Robotics. (2023). *ROS 2 Examples and Tutorials*. Retrieved from https://docs.ros.org/en/humble/Tutorials.html
- Quigley, M., Gerkey, B., & Smart, W. D. (2009). Programming robots with ROS: A practical introduction to the Robot Operating System. *Communications of the ACM*, 57(9), 82-91.
- Foote, T., Lalancette, C., & Quigley, J. (2016). ROS 2: Towards a robot platform for next generation robots. *Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 4698-4704.
- ROS 2 Documentation Working Group. (2023). *ROS 2 Developer Guide*. Retrieved from https://docs.ros.org/en/humble/How-To-Guides.html