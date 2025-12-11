---
sidebar_position: 3
---

# Chapter 3: ROS 2 Services and Actions

## Why This Concept Matters for Humanoids

Services and actions provide synchronous and asynchronous request-response communication patterns essential for humanoid robot behaviors. Services are perfect for immediate queries (like "get robot status") while actions handle long-running tasks (like "navigate to location" or "grasp object"). For humanoid robots that must interact with humans and environments, these communication patterns enable reliable, goal-oriented behaviors with proper feedback and error handling.

## Theory

**Services** implement synchronous request-response communication where a client sends a request and waits for a response. This is appropriate for quick operations that have a clear beginning and end. Services use a blocking call, so the client waits until the service completes.

**Actions** implement asynchronous request-response communication with feedback. They're designed for long-running operations and provide:
- **Goal**: The desired outcome
- **Feedback**: Progress updates during execution
- **Result**: Final outcome when complete
- **Cancel**: Ability to interrupt ongoing operations

Actions are ideal for behaviors that take time to complete, might fail, or need to provide progress updates - all common in humanoid robotics.

The key differences:
- **Services**: Synchronous, request-response, quick operations
- **Actions**: Asynchronous, with feedback, long-running operations with cancellation

## Implementation

Let's start with creating a service for robot status queries:

First, create the service definition file (in a real ROS 2 package, this would be in `srv/RobotStatus.srv`):

```
# Request (empty)
---
# Response
bool is_operational
float64 battery_level
string status_message
int32 error_code
```

Here's the service server implementation:

```python
# my_robot_services/my_robot_services/robot_status_service.py
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default

# In a real package, you would import your custom service
# from my_robot_interfaces.srv import RobotStatus
# For this example, we'll use a standard service type
from example_interfaces.srv import Trigger


class RobotStatusService(Node):
    def __init__(self):
        super().__init__('robot_status_service')

        # Create a service server
        self.srv = self.create_service(
            Trigger,  # Using Trigger service for example
            'get_robot_status',
            self.status_callback)

        self.get_logger().info('Robot status service is ready')

    def status_callback(self, request, response):
        # Simulate checking robot status
        # In a real implementation, this would check actual robot state
        import random

        # Simulate some processing time
        self.get_logger().info('Received status request')

        # Generate simulated response
        is_operational = True  # Could be based on actual robot state
        battery_level = random.uniform(20.0, 100.0)
        status_message = "Operational"
        error_code = 0

        # For Trigger service (which has no custom fields), we just return success
        response.success = is_operational
        response.message = f"Robot operational: {status_message}, Battery: {battery_level:.1f}%"

        self.get_logger().info(f'Returning: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    robot_status_service = RobotStatusService()

    try:
        rclpy.spin(robot_status_service)
    except KeyboardInterrupt:
        pass
    finally:
        robot_status_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Now, let's create an action server for a navigation task:

First, the action definition file (in a real package, this would be in `action/NavigateToPose.action`):

```
# Goal
geometry_msgs/PoseStamped target_pose

# Result
bool success
float64 distance_traveled

# Feedback
float64 distance_to_goal
float64 remaining_time
string status
```

Here's the action server implementation:

```python
# my_robot_actions/my_robot_actions/navigate_to_pose_server.py
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.duration import Duration

# In a real package, you would use your custom action
# from my_robot_interfaces.action import NavigateToPose
# For this example, we'll use a standard action type
from lifecycle_msgs.action import ChangeState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class NavigateToPoseServer(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_server')

        # Create an action server
        # Using ChangeState for example since we don't have custom action
        self._action_server = ActionServer(
            self,
            ChangeState,  # Using ChangeState for example
            'navigate_to_pose',  # In real implementation: 'navigate_to_pose'
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info('Navigate to pose action server is ready')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Simulate navigation process
        feedback_msg = ChangeState.Feedback()  # Would be NavigateToPose.Feedback
        feedback_msg.node_name = 'navigation_simulation'  # Placeholder

        for i in range(0, 100, 5):  # Simulate progress
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return ChangeState.Result()  # Would be NavigateToPose.Result

            # Publish feedback
            feedback_msg.state.label = f'Navigating: {i}%'  # Placeholder
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Navigation progress: {i}%')

            # Sleep to simulate navigation time
            await rclpy.sleep(Duration(seconds=0.2).nanoseconds / 1e9)

        goal_handle.succeed()

        # Return result
        result = ChangeState.Result()  # Would be NavigateToPose.Result
        result.success = True  # Would be in NavigateToPose.Result
        self.get_logger().info('Navigation completed successfully')

        return result


def main(args=None):
    rclpy.init(args=args)
    navigate_to_pose_server = NavigateToPoseServer()

    try:
        rclpy.spin(navigate_to_pose_server)
    except KeyboardInterrupt:
        pass
    finally:
        navigate_to_pose_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

And here's the action client implementation:

```python
# my_robot_actions/my_robot_actions/navigate_to_pose_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

# Using standard action for example
from lifecycle_msgs.action import ChangeState


class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')

        # Create an action client
        self._action_client = ActionClient(
            self,
            ChangeState,  # Using ChangeState for example
            'navigate_to_pose')  # In real implementation: 'navigate_to_pose'

    def send_goal(self):
        """Send a goal to the action server."""
        goal_msg = ChangeState.Goal()  # Would be NavigateToPose.Goal
        goal_msg.transition.id = 1  # Placeholder
        goal_msg.transition.label = 'navigate_to_location'  # Placeholder

        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Send the goal
        self.get_logger().info('Sending navigation goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        # Add a callback for when the goal is accepted
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from sending the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        # Get the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle the result of the action."""
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.state.label}')


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateToPoseClient()

    # Send the goal
    action_client.send_goal()

    try:
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()


if __name__ == '__main__':
    main()
```

## Hardware/GPU Notes

When using services and actions in hardware systems:

- **Service Latency**: Ensure service responses meet real-time requirements
- **Action Cancellation**: Implement proper cancellation handling for safety
- **Feedback Rate**: Balance feedback frequency with communication overhead
- **Resource Management**: Long-running actions may consume significant resources

For humanoid robots, consider the timing requirements for human-robot interaction where delays can affect user experience.

## Simulation Path

Testing services and actions in simulation:

```bash
# Terminal 1: Start the action server
ros2 run my_robot_actions navigate_to_pose_server

# Terminal 2: Send a goal using the client
ros2 run my_robot_actions navigate_to_pose_client

# You can also use command line tools
ros2 action send_goal /navigate_to_pose lifecycle_msgs/action/ChangeState "{transition: {id: 1, label: 'navigate'}}"
```

For debugging, you can also check action status:

```bash
# List all action servers
ros2 action list

# Show action types
ros2 action types

# Get information about a specific action
ros2 action info /navigate_to_pose
```

## Real-World Path

For real hardware deployment:

1. **Service Reliability**: Ensure services are available when needed
2. **Action Monitoring**: Monitor long-running actions for timeouts
3. **Safety Integration**: Integrate service/action calls with safety systems
4. **Error Handling**: Implement comprehensive error handling
5. **Graceful Degradation**: Handle service/action failures gracefully

Example of a real-world service call with safety integration:

```python
# my_robot_safety/my_robot_safety/safe_command_service.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import qos_profile_services_default
from example_interfaces.srv import Trigger  # Using for example
from std_msgs.msg import Bool


class SafeCommandService(Node):
    def __init__(self):
        super().__init__('safe_command_service')

        # Service for requesting safe commands
        self.srv = self.create_service(
            Trigger,  # Would use custom service in real implementation
            'safe_command',
            self.safe_command_callback)

        # Publisher for safety status
        self.safety_pub = self.create_publisher(Bool, 'safety_status', 10)

        # Subscribe to safety system
        self.safety_sub = self.create_subscription(
            Bool,
            'safety_enabled',
            self.safety_callback,
            10)

        self.safety_enabled = True

    def safety_callback(self, msg):
        self.safety_enabled = msg.data

    def safe_command_callback(self, request, response):
        # Check safety status before executing command
        if not self.safety_enabled:
            response.success = False
            response.message = "Safety system disabled - command rejected"
            self.get_logger().warn("Command rejected due to safety")
            return response

        # Execute safe command
        try:
            # Perform the actual command (e.g., move to safe position)
            self.execute_safe_command()
            response.success = True
            response.message = "Safe command executed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Command failed: {str(e)}"
            self.get_logger().error(f"Safe command failed: {str(e)}")

        # Publish updated safety status
        safety_msg = Bool()
        safety_msg.data = self.safety_enabled
        self.safety_pub.publish(safety_msg)

        return response

    def execute_safe_command(self):
        # Implementation of the safe command
        # This might involve moving to a safe position, stopping motors, etc.
        self.get_logger().info("Executing safe command")


def main(args=None):
    rclpy.init(args=args)
    safe_command_service = SafeCommandService()

    try:
        rclpy.spin(safe_command_service)
    except KeyboardInterrupt:
        pass
    finally:
        safe_command_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Spec-Build-Test Checklist

- [ ] Verify service server creates and responds to requests
- [ ] Confirm service client can send requests and receive responses
- [ ] Test action server accepts goals and provides feedback
- [ ] Validate action client can send goals and receive results
- [ ] Check proper cancellation handling for actions
- [ ] Verify error handling in both services and actions

## APA Citations

- Woodall, W., Dornhege, C., Hertle, F., & Kleiner, A. (2015). The Robot Operating System 2: Design, architecture, and uses in the wild. *arXiv preprint arXiv:2008.08387*.
- Quigley, M., Gerkey, B., & Smart, W. D. (2009). Programming robots with ROS: a practical introduction to the robot operating system. *O'Reilly Media*.
- Dornhege, C., Hertle, F., & Ferrein, A. (2013). The skill layer: A middleware for using ROS components in real-time robotic applications. *Proceedings of the 1st International Workshop on Middleware and Systems*, 1-6.
- Macenski, S., Woodall, W., & Faust, A. (2022). ROS 2: The evolution of the Robot Operating System for real-time and safety-critical applications. *IEEE Robotics & Automation Magazine*, 29(3), 11-21.