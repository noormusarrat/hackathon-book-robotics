---
sidebar_position: 9
title: "Module 4 Examples: VLA Robotics Code Samples"
description: "Example code implementations for Vision-Language-Action robotics systems"
---

# Module 4 Examples: VLA Robotics Code Samples

## Overview

This section contains complete, runnable code examples that demonstrate the implementation of Vision-Language-Action (VLA) robotics systems. Each example builds upon the concepts covered in the module chapters and provides practical implementations that can be used as starting points for your own projects.

## Example 1: Basic Voice Command Node

This example demonstrates a simple voice command system that listens for audio input and publishes movement commands.

### File: `voice_command_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import speech_recognition as sr
import threading

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')

        # Create publisher for movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Start voice listening thread
        self.listening = True
        self.voice_thread = threading.Thread(target=self.listen_for_commands)
        self.voice_thread.daemon = True
        self.voice_thread.start()

        self.get_logger().info('Voice Command Node initialized')

    def listen_for_commands(self):
        """Listen for voice commands in a separate thread"""
        with self.microphone as source:
            while self.listening:
                try:
                    self.get_logger().info('Listening for commands...')
                    audio = self.recognizer.listen(source, timeout=1.0, phrase_time_limit=3.0)

                    # Recognize speech
                    command = self.recognizer.recognize_google(audio).lower()
                    self.get_logger().info(f'Heard: {command}')

                    # Process command
                    self.process_command(command)

                except sr.WaitTimeoutError:
                    # No speech detected, continue listening
                    pass
                except sr.UnknownValueError:
                    self.get_logger().info('Could not understand audio')
                except sr.RequestError as e:
                    self.get_logger().error(f'Error: {e}')

    def process_command(self, command):
        """Process recognized voice command"""
        twist = Twist()

        if 'forward' in command or 'ahead' in command:
            twist.linear.x = 0.5
            self.get_logger().info('Moving forward')
        elif 'backward' in command or 'back' in command:
            twist.linear.x = -0.5
            self.get_logger().info('Moving backward')
        elif 'left' in command:
            twist.angular.z = 0.5
            self.get_logger().info('Turning left')
        elif 'right' in command:
            twist.angular.z = -0.5
            self.get_logger().info('Turning right')
        elif 'stop' in command or 'halt' in command:
            # All velocities remain zero
            self.get_logger().info('Stopping')
        else:
            self.get_logger().info(f'Unknown command: {command}')
            return

        # Publish the command
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down voice command node')
        node.listening = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Setup and Running

1. Install required packages:
```bash
pip3 install speechrecognition pyaudio
```

2. Run the node:
```bash
ros2 run your_package voice_command_node.py
```

---

## Example 2: Natural Language Understanding with Pattern Matching

This example shows how to implement basic natural language understanding using pattern matching.

### File: `nlu_pattern_matcher.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re
import json

class NLUProcessor(Node):
    def __init__(self):
        super().__init__('nlu_processor')

        # Subscribe to voice commands
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.command_callback, 10
        )

        # Publish structured commands
        self.structured_pub = self.create_publisher(
            String, 'structured_command', 10
        )

        # Define patterns for different intents
        self.patterns = {
            'navigation': [
                (r'go\s+(?P<direction>forward|backward|left|right)', self.handle_navigation),
                (r'move\s+(?P<direction>forward|backward|left|right)', self.handle_navigation),
                (r'go\s+to\s+(?P<location>\w+)', self.handle_location_navigation),
            ],
            'manipulation': [
                (r'pick\s+up\s+(?P<object>.+)', self.handle_pick_up),
                (r'grab\s+(?P<object>.+)', self.handle_pick_up),
                (r'put\s+(?P<object>.+)\s+down', self.handle_place),
                (r'place\s+(?P<object>.+)', self.handle_place),
            ],
            'information': [
                (r'what.*time', self.handle_time_request),
                (r'hello|hi|hey', self.handle_greeting),
            ]
        }

        self.get_logger().info('NLU Processor initialized')

    def command_callback(self, msg):
        """Process incoming voice command"""
        command = msg.data.lower()
        self.get_logger().info(f'Processing: {command}')

        # Try to match patterns
        for intent, pattern_list in self.patterns.items():
            for pattern, handler in pattern_list:
                match = re.search(pattern, command)
                if match:
                    result = handler(match)
                    if result:
                        # Publish structured command
                        structured_msg = String()
                        structured_msg.data = json.dumps(result)
                        self.structured_pub.publish(structured_msg)
                        return

        # If no pattern matches, publish unknown command
        unknown_msg = String()
        unknown_msg.data = json.dumps({
            'intent': 'unknown',
            'original_command': command,
            'confidence': 0.0
        })
        self.structured_pub.publish(unknown_msg)

    def handle_navigation(self, match):
        """Handle navigation commands"""
        direction = match.group('direction')
        return {
            'intent': 'navigation',
            'action': 'move_directionally',
            'parameters': {'direction': direction},
            'confidence': 0.9
        }

    def handle_location_navigation(self, match):
        """Handle navigation to specific location"""
        location = match.group('location')
        return {
            'intent': 'navigation',
            'action': 'move_to_location',
            'parameters': {'location': location},
            'confidence': 0.8
        }

    def handle_pick_up(self, match):
        """Handle pick up commands"""
        obj = match.group('object').strip()
        return {
            'intent': 'manipulation',
            'action': 'pick_up',
            'parameters': {'object': obj},
            'confidence': 0.85
        }

    def handle_place(self, match):
        """Handle place commands"""
        obj = match.group('object').strip()
        return {
            'intent': 'manipulation',
            'action': 'place',
            'parameters': {'object': obj},
            'confidence': 0.85
        }

    def handle_time_request(self, match):
        """Handle time requests"""
        from datetime import datetime
        current_time = datetime.now().strftime("%H:%M")
        return {
            'intent': 'information',
            'action': 'speak',
            'parameters': {'text': f'The current time is {current_time}'},
            'confidence': 0.95
        }

    def handle_greeting(self, match):
        """Handle greetings"""
        return {
            'intent': 'information',
            'action': 'speak',
            'parameters': {'text': 'Hello! How can I help you today?'},
            'confidence': 0.95
        }

def main(args=None):
    rclpy.init(args=args)
    node = NLUProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Example 3: Voice-to-Action Pipeline Integration

This example demonstrates how to integrate voice processing, NLU, and action execution.

### File: `vla_pipeline.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import json
import time
from enum import Enum

class PipelineState(Enum):
    IDLE = 1
    PROCESSING = 2
    EXECUTING = 3
    ERROR = 4

class VLAPipeline(Node):
    def __init__(self):
        super().__init__('vla_pipeline')

        # Initialize pipeline state
        self.state = PipelineState.IDLE

        # Publishers and subscribers
        self.voice_sub = self.create_subscription(
            String, 'voice_command', self.voice_callback, 10
        )
        self.nlu_sub = self.create_subscription(
            String, 'structured_command', self.nlu_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_pub = self.create_publisher(String, 'pipeline_feedback', 10)

        # Action execution timer
        self.action_timer = None
        self.action_start_time = 0
        self.action_duration = 0

        self.get_logger().info('VLA Pipeline initialized')

    def voice_callback(self, msg):
        """Handle voice commands"""
        if self.state == PipelineState.IDLE:
            self.state = PipelineState.PROCESSING
            self.get_logger().info(f'Received voice command: {msg.data}')

            # Publish for NLU processing
            # In a real system, this would be handled by the NLU node
            self.process_command_locally(msg.data)

    def nlu_callback(self, msg):
        """Handle structured commands from NLU"""
        try:
            command_data = json.loads(msg.data)
            self.execute_action(command_data)
        except json.JSONDecodeError:
            self.get_logger().error('Invalid JSON in structured command')

    def process_command_locally(self, command_text):
        """Local processing of command (for demonstration)"""
        # Simple command processing - in practice, this would call the NLU system
        command_data = self.simple_nlu(command_text)
        if command_data:
            self.execute_action(command_data)

    def simple_nlu(self, command):
        """Simple natural language understanding for demo"""
        command_lower = command.lower()

        if 'forward' in command_lower or 'ahead' in command_lower:
            return {
                'intent': 'navigation',
                'action': 'move_linear',
                'parameters': {'direction': 'forward', 'duration': 2.0, 'speed': 0.5}
            }
        elif 'backward' in command_lower or 'back' in command_lower:
            return {
                'intent': 'navigation',
                'action': 'move_linear',
                'parameters': {'direction': 'backward', 'duration': 2.0, 'speed': 0.5}
            }
        elif 'turn left' in command_lower or 'left' in command_lower:
            return {
                'intent': 'navigation',
                'action': 'turn',
                'parameters': {'direction': 'left', 'angle': 90, 'speed': 0.5}
            }
        elif 'turn right' in command_lower or 'right' in command_lower:
            return {
                'intent': 'navigation',
                'action': 'turn',
                'parameters': {'direction': 'right', 'angle': 90, 'speed': 0.5}
            }
        else:
            return {
                'intent': 'unknown',
                'action': 'none',
                'parameters': {}
            }

    def execute_action(self, command_data):
        """Execute the planned action"""
        self.state = PipelineState.EXECUTING
        intent = command_data.get('intent', 'unknown')

        if intent == 'navigation':
            action = command_data.get('action', '')
            params = command_data.get('parameters', {})

            if action == 'move_linear':
                self.execute_linear_movement(params)
            elif action == 'turn':
                self.execute_turn(params)
        elif intent == 'unknown':
            self.get_logger().info('Unknown command, cannot execute')
            self.state = PipelineState.IDLE
            return
        else:
            self.get_logger().info(f'Intent {intent} not implemented yet')
            self.state = PipelineState.IDLE
            return

        # Provide feedback
        feedback_msg = String()
        feedback_msg.data = f"Executing: {command_data.get('action', 'unknown')}"
        self.feedback_pub.publish(feedback_msg)

    def execute_linear_movement(self, params):
        """Execute linear movement"""
        direction = params.get('direction', 'forward')
        duration = params.get('duration', 2.0)
        speed = params.get('speed', 0.5)

        twist = Twist()
        if direction == 'forward':
            twist.linear.x = speed
        else:
            twist.linear.x = -speed

        self.get_logger().info(f'Moving {direction} at speed {speed} for {duration}s')

        # Publish command and schedule stop
        self.cmd_vel_pub.publish(twist)

        # Stop after duration
        timer = self.create_timer(duration, self.stop_movement)
        self.action_timer = timer

    def execute_turn(self, params):
        """Execute turning movement"""
        direction = params.get('direction', 'left')
        angle = params.get('angle', 90)
        speed = params.get('speed', 0.5)

        # Convert angle to time (simplified - in reality would use feedback)
        # Assuming 90 degrees takes 2 seconds at speed 0.5
        duration = (angle / 90.0) * 2.0

        twist = Twist()
        if direction == 'left':
            twist.angular.z = speed
        else:
            twist.angular.z = -speed

        self.get_logger().info(f'Turning {direction} for {duration}s')

        # Publish command and schedule stop
        self.cmd_vel_pub.publish(twist)

        # Stop after duration
        timer = self.create_timer(duration, self.stop_movement)
        self.action_timer = timer

    def stop_movement(self):
        """Stop robot movement"""
        if self.action_timer:
            self.action_timer.cancel()

        # Publish zero velocity to stop
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info('Movement stopped')
        self.state = PipelineState.IDLE

def main(args=None):
    rclpy.init(args=args)
    pipeline = VLAPipeline()

    try:
        rclpy.spin(pipeline)
    except KeyboardInterrupt:
        pipeline.get_logger().info('Shutting down VLA pipeline')
    finally:
        # Stop any ongoing movement
        if pipeline.state == PipelineState.EXECUTING:
            pipeline.stop_movement()
        pipeline.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Example 4: Human-Robot Interaction Manager

This example demonstrates social interaction behaviors and user state management.

### File: `hri_manager.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import math
from enum import Enum

class UserState(Enum):
    UNKNOWN = 0
    APPROACHING = 1
    ENGAGED = 2
    INTERACTING = 3
    LEAVING = 4

class HRIManager(Node):
    def __init__(self):
        super().__init__('hri_manager')

        # User tracking
        self.users = {}  # Dictionary to track multiple users
        self.personal_space_radius = 1.0  # meters
        self.social_space_radius = 2.0    # meters

        # Publishers and subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        self.voice_sub = self.create_subscription(
            String, 'voice_command', self.voice_callback, 10
        )
        self.feedback_pub = self.create_publisher(String, 'hri_feedback', 10)

        # Timer for user state monitoring
        self.user_monitor_timer = self.create_timer(0.5, self.monitor_users)

        self.get_logger().info('HRI Manager initialized')

    def laser_callback(self, msg):
        """Process laser scan to detect users"""
        # Simple clustering to detect people
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        # Convert laser readings to Cartesian coordinates
        points = []
        for i, range_val in enumerate(ranges):
            if not math.isnan(range_val) and range_val < 3.0:  # Only consider points within 3m
                angle = angle_min + i * angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                points.append(Point(x=x, y=y, z=0.0))

        # Group nearby points into potential users
        clusters = self.cluster_points(points, threshold=0.5)

        # Update user tracking
        current_users = {}
        for i, cluster in enumerate(clusters):
            # Calculate cluster center
            avg_x = sum(p.x for p in cluster) / len(cluster)
            avg_y = sum(p.y for p in cluster) / len(cluster)
            center = Point(x=avg_x, y=avg_y, z=0.0)

            user_id = f'user_{i}'
            current_users[user_id] = center

        # Update tracked users
        self.update_tracked_users(current_users)

    def cluster_points(self, points, threshold=0.5):
        """Simple clustering algorithm to group nearby points"""
        clusters = []

        for point in points:
            # Find if this point belongs to an existing cluster
            assigned = False
            for cluster in clusters:
                # Calculate distance to cluster center
                center_x = sum(p.x for p in cluster) / len(cluster)
                center_y = sum(p.y for p in cluster) / len(cluster)

                dist = math.sqrt((point.x - center_x)**2 + (point.y - center_y)**2)

                if dist < threshold:
                    cluster.append(point)
                    assigned = True
                    break

            # If not assigned to any cluster, create a new one
            if not assigned:
                clusters.append([point])

        return clusters

    def update_tracked_users(self, current_users):
        """Update tracking information for users"""
        for user_id, position in current_users.items():
            # Calculate distance from robot (assuming robot is at origin)
            distance = math.sqrt(position.x**2 + position.y**2)

            # Determine user state based on distance
            if distance < self.personal_space_radius:
                state = UserState.INTERACTING
            elif distance < self.social_space_radius:
                state = UserState.ENGAGED
            else:
                state = UserState.APPROACHING

            # Update or create user tracking
            if user_id not in self.users:
                self.users[user_id] = {
                    'position': position,
                    'state': state,
                    'last_seen': self.get_clock().now().nanoseconds
                }
                self.handle_user_appearance(user_id, state)
            else:
                old_state = self.users[user_id]['state']
                self.users[user_id]['position'] = position
                self.users[user_id]['last_seen'] = self.get_clock().now().nanoseconds

                if old_state != state:
                    self.handle_user_state_change(user_id, old_state, state)

        # Remove users not seen for a while
        current_time = self.get_clock().now().nanoseconds
        users_to_remove = []
        for user_id, user_data in self.users.items():
            time_since_seen = (current_time - user_data['last_seen']) / 1e9  # Convert to seconds
            if time_since_seen > 5.0:  # Remove if not seen for 5 seconds
                users_to_remove.append(user_id)

        for user_id in users_to_remove:
            old_state = self.users[user_id]['state']
            del self.users[user_id]
            self.handle_user_disappearance(user_id, old_state)

    def handle_user_appearance(self, user_id, state):
        """Handle when a new user appears"""
        self.get_logger().info(f'New user detected: {user_id}, state: {state.name}')

        # Respond based on state
        if state == UserState.INTERACTING:
            self.provide_welcome_message(user_id)
        elif state == UserState.ENGAGED:
            self.provide_attention_message(user_id)

    def handle_user_state_change(self, user_id, old_state, new_state):
        """Handle changes in user state"""
        self.get_logger().info(f'User {user_id} state changed: {old_state.name} -> {new_state.name}')

        if old_state == UserState.APPROACHING and new_state == UserState.ENGAGED:
            self.provide_attention_message(user_id)
        elif old_state == UserState.ENGAGED and new_state == UserState.INTERACTING:
            self.provide_interaction_ready_message(user_id)
        elif old_state == UserState.INTERACTING and new_state in [UserState.ENGAGED, UserState.APPROACHING]:
            self.provide_interaction_ended_message(user_id)

    def handle_user_disappearance(self, user_id, old_state):
        """Handle when a user disappears"""
        self.get_logger().info(f'User {user_id} disappeared (was {old_state.name})')

        if old_state == UserState.INTERACTING:
            self.provide_goodbye_message(user_id)

    def provide_welcome_message(self, user_id):
        """Provide welcome message when user enters personal space"""
        feedback_msg = String()
        feedback_msg.data = f"GREETING: Hello {user_id}! I'm ready to help. What would you like me to do?"
        self.feedback_pub.publish(feedback_msg)

    def provide_attention_message(self, user_id):
        """Provide attention message when user enters social space"""
        feedback_msg = String()
        feedback_msg.data = f"ATTENTION: I see you {user_id}. Let me know if you need assistance."
        self.feedback_pub.publish(feedback_msg)

    def provide_interaction_ready_message(self, user_id):
        """Provide message when ready for direct interaction"""
        feedback_msg = String()
        feedback_msg.data = f"INTERACTION_READY: I'm ready to talk with you {user_id}. How can I help?"
        self.feedback_pub.publish(feedback_msg)

    def provide_interaction_ended_message(self, user_id):
        """Provide message when interaction ends"""
        feedback_msg = String()
        feedback_msg.data = f"INTERACTION_ENDED: Thank you for talking with me {user_id}."
        self.feedback_pub.publish(feedback_msg)

    def provide_goodbye_message(self, user_id):
        """Provide goodbye message when user leaves"""
        feedback_msg = String()
        feedback_msg.data = f"GOODBYE: Goodbye {user_id}. Have a great day!"
        self.feedback_pub.publish(feedback_msg)

    def voice_callback(self, msg):
        """Handle voice commands from users"""
        command = msg.data.lower()

        # Check if any user is in interaction state
        interacting_users = [uid for uid, data in self.users.items()
                            if data['state'] == UserState.INTERACTING]

        if interacting_users:
            # Acknowledge the command
            feedback_msg = String()
            feedback_msg.data = f"ACKNOWLEDGMENT: I heard you. Processing command: {command}"
            self.feedback_pub.publish(feedback_msg)

    def monitor_users(self):
        """Monitor user states and trigger appropriate behaviors"""
        # This runs periodically to monitor users
        active_users = {uid: data for uid, data in self.users.items()
                       if data['state'] in [UserState.ENGAGED, UserState.INTERACTING]}

        if active_users:
            self.get_logger().debug(f'Active users: {len(active_users)}')

def main(args=None):
    rclpy.init(args=args)
    hri_manager = HRIManager()

    try:
        rclpy.spin(hri_manager)
    except KeyboardInterrupt:
        hri_manager.get_logger().info('Shutting down HRI manager')
    finally:
        hri_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Example 5: Complete VLA System Launch File

### File: `vla_system.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Voice processing node
        Node(
            package='your_robot_package',
            executable='voice_command_node',
            name='voice_command_node',
            output='screen',
            parameters=[
                # Add any parameters needed
            ]
        ),

        # NLU processing node
        Node(
            package='your_robot_package',
            executable='nlu_processor',
            name='nlu_processor',
            output='screen'
        ),

        # VLA pipeline node
        Node(
            package='your_robot_package',
            executable='vla_pipeline',
            name='vla_pipeline',
            output='screen'
        ),

        # HRI manager node
        Node(
            package='your_robot_package',
            executable='hri_manager',
            name='hri_manager',
            output='screen'
        )
    ])
```

---

## Example 6: Configuration File

### File: `vla_config.yaml`

```yaml
voice_to_action:
  voice_processing:
    sample_rate: 16000
    chunk_size: 1024
    vad_aggressiveness: 2
    energy_threshold: 300

  nlu:
    confidence_threshold: 0.7
    max_ambiguity: 0.3

  action_planning:
    max_planning_time: 5.0
    enable_validation: true

  hri:
    personal_space_radius: 1.0
    social_space_radius: 2.0
    public_space_radius: 4.0

  pipeline:
    response_timeout: 10.0
    enable_feedback: true
    feedback_language: "en"
```

---

## Running the Examples

To run these examples:

1. **Set up your ROS 2 workspace:**
```bash
mkdir -p ~/vla_examples_ws/src
cd ~/vla_examples_ws/src
git clone <your_robot_package>
cd ..
colcon build
source install/setup.bash
```

2. **Install required Python packages:**
```bash
pip3 install speechrecognition pyaudio
```

3. **Launch the complete system:**
```bash
ros2 launch your_robot_package vla_system.launch.py
```

4. **Test with voice commands:**
```bash
# Manually publish commands for testing
ros2 topic pub /voice_command std_msgs/String "data: 'move forward'"
```

## Important Notes

- These examples are simplified for educational purposes
- In production systems, add proper error handling and safety checks
- Consider privacy implications when using voice recognition
- Adapt the configuration parameters to your specific robot platform
- Test thoroughly in simulation before deploying on physical robots

For more advanced implementations, consider integrating with actual speech-to-text APIs, computer vision systems, and proper action planners.