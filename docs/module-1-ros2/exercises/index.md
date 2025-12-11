---
title: Module 1 Exercises - ROS 2 Fundamentals
sidebar_position: 1
---

# Module 1 Exercises: ROS 2 Fundamentals

## Exercise 1: Basic Publisher-Subscriber Pattern

**Difficulty**: Easy
**Type**: Code
**Required Tools**: ROS 2 Humble, Python or C++
**Estimated Time**: 45-60 minutes

### Objective
Create a simple publisher-subscriber pair that demonstrates the basic communication pattern in ROS 2.

### Instructions
1. Create a new ROS 2 package called `my_robot_tutorials`
2. Implement a publisher node that publishes a `std_msgs/String` message containing "Hello, ROS 2!" every 2 seconds
3. Implement a subscriber node that listens to this topic and prints the received message to the console
4. Use proper node lifecycle management with error handling
5. Add parameters to control the publishing rate and message content

### Validation Criteria
- Publisher and subscriber nodes run without errors
- Messages are successfully published and received
- Parameter changes affect node behavior as expected
- Proper error handling is implemented
- Code follows ROS 2 best practices

### Implementation Steps
1. Create package: `ros2 pkg create --build-type ament_python my_robot_tutorials`
2. Create publisher script in `my_robot_tutorials/my_robot_tutorials/hello_publisher.py`
3. Create subscriber script in `my_robot_tutorials/my_robot_tutorials/hello_subscriber.py`
4. Add executables to `setup.py`
5. Test with `ros2 run` commands

## Exercise 2: Service Implementation

**Difficulty**: Medium
**Type**: Code
**Required Tools**: ROS 2 Humble, Python or C++
**Estimated Time**: 60-90 minutes

### Objective
Implement a service that performs a calculation or provides information about the robot's state.

### Instructions
1. Create a custom service definition (`.srv` file) that takes two numbers and returns their sum and product
2. Implement a service server that handles the calculation
3. Implement a service client that calls the service with different inputs
4. Add error handling for invalid inputs
5. Test the service with various input combinations

### Validation Criteria
- Custom service definition compiles successfully
- Service server responds correctly to requests
- Service client receives and processes responses
- Error handling works for invalid inputs
- Service follows ROS 2 conventions

### Implementation Steps
1. Create `srv/AddMultiply.srv` with appropriate request/response fields
2. Generate the service definition
3. Implement server node that processes requests
4. Implement client node that sends requests
5. Test with various input values

## Exercise 3: Action Server for Robot Movement

**Difficulty**: Hard
**Type**: Code
**Required Tools**: ROS 2 Humble, Python or C++
**Estimated Time**: 90-120 minutes

### Objective
Create an action server that simulates moving a robot to a target position with feedback and goal management.

### Instructions
1. Define a custom action (`.action` file) for robot movement with target position and progress feedback
2. Implement an action server that simulates movement toward a target
3. Implement an action client that sends movement goals and monitors progress
4. Include proper goal preemption and cancellation handling
5. Add realistic simulation of movement time and potential failures

### Validation Criteria
- Action server correctly handles goals, feedback, and results
- Action client can send goals and receive feedback
- Goal preemption and cancellation work properly
- Error simulation behaves realistically
- Action follows ROS 2 best practices

### Implementation Steps
1. Create `action/MoveToPosition.action` with appropriate fields
2. Generate the action definition
3. Implement action server with simulation logic
4. Implement action client with goal monitoring
5. Test goal preemption and cancellation scenarios

## Exercise 4: Launch File Configuration

**Difficulty**: Medium
**Type**: Configuration
**Required Tools**: ROS 2 Humble, Launch system
**Estimated Time**: 45-60 minutes

### Objective
Create a comprehensive launch file that starts multiple nodes with different configurations.

### Instructions
1. Create a launch file that starts the publisher and subscriber from Exercise 1
2. Add parameters to configure the behavior of each node
3. Include conditional startup based on launch arguments
4. Add nodes for the service server and client from Exercise 2
5. Include proper error handling and cleanup

### Validation Criteria
- Launch file starts all specified nodes successfully
- Parameters are correctly passed to nodes
- Conditional startup works as expected
- All nodes can be properly shut down
- Launch file follows ROS 2 best practices

### Implementation Steps
1. Create `launch/multi_node_system.launch.py`
2. Define launch arguments for configuration
3. Create node definitions with parameter overrides
4. Test launch file with different argument combinations

## Exercise 5: Node Parameters and Configuration

**Difficulty**: Medium
**Type**: Code
**Required Tools**: ROS 2 Humble, Parameter system
**Estimated Time**: 60-75 minutes

### Objective
Implement parameter handling in a ROS 2 node with runtime updates and configuration files.

### Instructions
1. Create a node that uses multiple parameters for configuration
2. Implement parameter validation and callbacks for runtime updates
3. Create a YAML configuration file with different parameter sets
4. Add command-line parameter overrides
5. Include parameter change logging and validation

### Validation Criteria
- Parameters are properly declared and validated
- Runtime parameter updates work correctly
- Configuration files are loaded successfully
- Parameter callbacks execute as expected
- Error handling covers invalid parameter values

### Implementation Steps
1. Create parameter handling node with callbacks
2. Define configuration YAML files
3. Test parameter updates during runtime
4. Validate parameter constraints

## Exercise 6: Quality of Service (QoS) Configuration

**Difficulty**: Hard
**Type**: Code
**Required Tools**: ROS 2 Humble, QoS system
**Estimated Time**: 75-90 minutes

### Objective
Implement different QoS profiles to understand their impact on communication reliability and performance.

### Instructions
1. Create publisher-subscriber pairs with different QoS profiles (reliable vs. best effort)
2. Test communication under different network conditions
3. Implement QoS adaptation based on network performance
4. Measure and compare performance metrics between profiles
5. Document the trade-offs between reliability and performance

### Validation Criteria
- Different QoS profiles are correctly implemented
- Performance differences are measurable and documented
- QoS adaptation logic works as expected
- Network performance metrics are collected
- Trade-offs are clearly explained

### Implementation Steps
1. Create nodes with different QoS configurations
2. Implement performance measurement tools
3. Test under various simulated network conditions
4. Document findings and recommendations

## Exercise 7: TF2 Transformations for Humanoid Robot

**Difficulty**: Hard
**Type**: Code
**Required Tools**: ROS 2 Humble, TF2 library
**Estimated Time**: 120-150 minutes

### Objective
Implement a transform tree for a simplified humanoid robot model with proper coordinate frame management.

### Instructions
1. Create a URDF model of a simplified humanoid robot with basic joint structure
2. Implement a node that publishes transforms for the robot's joints
3. Use TF2 to query transforms between different parts of the robot
4. Implement forward and inverse kinematics for a simple limb
5. Visualize the transform tree in RViz2

### Validation Criteria
- Transform tree is properly structured and published
- TF2 queries return correct transformations
- Kinematics calculations are accurate
- Visualization shows correct robot pose
- Transform tree follows ROS 2 conventions

### Implementation Steps
1. Create simplified URDF model of humanoid robot
2. Implement joint state publisher
3. Use TF2 broadcaster to publish transforms
4. Implement TF2 listener for querying transforms
5. Test with RViz2 visualization

## Exercise 8: Robot State Publisher Integration

**Difficulty**: Medium
**Type**: Integration
**Required Tools**: ROS 2 Humble, Robot State Publisher
**Estimated Time**: 60-75 minutes

### Objective
Integrate robot state publisher with a URDF model to publish joint transforms automatically.

### Instructions
1. Create a simple URDF model of a robot arm or leg
2. Implement a joint state publisher that simulates joint positions
3. Use robot_state_publisher to automatically publish the transform tree
4. Verify transforms in RViz2
5. Test with different joint configurations

### Validation Criteria
- URDF model loads correctly
- Joint states are published at appropriate rate
- Transform tree updates in real-time
- Robot model displays correctly in RViz2
- Joint movements are visualized properly

### Implementation Steps
1. Create URDF model with proper joint definitions
2. Implement joint state publisher node
3. Launch robot_state_publisher with URDF
4. Visualize in RViz2 with robot model

## APA Citations

- Open Robotics. (2023). *ROS 2 Tutorials: Exercises and Examples*. Retrieved from https://docs.ros.org/en/humble/Tutorials.html
- Siciliano, B., & Khatib, O. (2016). *Springer handbook of robotics*. Springer.
- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). *Robot modeling and control*. John Wiley & Sons.
- Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic robotics*. MIT Press.