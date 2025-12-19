---
sidebar_position: 2
---

# Connecting Python Agents to ROS 2 Controllers with `rclpy`

`rclpy` is the Python client library for ROS 2, providing a powerful and flexible interface for developing robotic applications. It enables Python developers to create ROS 2 nodes, communicate with other nodes, and integrate high-level intelligence and AI/ML models into their robotic systems. For humanoid robots, `rclpy` is an invaluable tool for building complex agents that orchestrate perception, planning, and control.

## 1. Introduction to `rclpy` for Robotics Agents

Python's ease of use, extensive libraries for AI and data science, and growing performance make it an ideal language for developing robotic agents. `rclpy` bridges the gap between the Python ecosystem and the ROS 2 framework, allowing engineers to:

-   **Rapidly Prototype:** Quickly develop and test robot behaviors and algorithms.
-   **Integrate AI/ML:** Seamlessly incorporate deep learning models (e.g., PyTorch, TensorFlow) for advanced perception, decision-making, and natural language processing.
-   **High-Level Control:** Implement complex task planning, state machines, and behavior trees in a readable and maintainable way.

This section will guide you through building robust Python-based ROS 2 agents using `rclpy`.

## 2. Deep Dive into `rclpy` Core Concepts

Building upon the ROS 2 fundamentals discussed in the previous chapter, we'll explore how these concepts are realized in Python with `rclpy`.

### 2.1. Node Initialization and Management

Every `rclpy` application starts with creating a `Node` object.

-   **`rclpy.init(args=None)`:** Initializes the ROS 2 client library for Python. This must be called before any other `rclpy` functions.
-   **`rclpy.create_node(node_name)`:** Creates a new ROS 2 node instance.
-   **`rclpy.spin(node)`:** Blocks until the node is shut down, processing callbacks as messages arrive or timers expire. For asynchronous processing, `rclpy.spin_once()` or `rclpy.executors` are used.
-   **`node.destroy_node()`:** Cleans up the node resources.
-   **`rclpy.shutdown()`:** Shuts down the `rclpy` library.

**Example: Basic `rclpy` Node**
```python
import rclpy
from rclpy.node import Node

class MyAgentNode(Node):
    def __init__(self):
        super().__init__('my_agent_node') # Initialize node with a name
        self.get_logger().info("MyAgentNode has been initialized!")

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    agent_node = MyAgentNode() # Create the node
    rclpy.spin(agent_node) # Keep node alive
    agent_node.destroy_node() # Destroy node on shutdown
    rclpy.shutdown() # Shut down rclpy

if __name__ == '__main__':
    main()
```

### 2.2. Publisher/Subscriber Patterns in Python

The publish-subscribe model is the backbone of ROS 2 communication.

-   **Creating `Publisher` and `Subscriber` Objects:**
    -   `node.create_publisher(msg_type, topic_name, qos_profile)`
    -   `node.create_subscription(msg_type, topic_name, callback_function, qos_profile)`
-   **Handling Callbacks and Asynchronous Processing:** Callbacks are functions that execute when a new message arrives on a subscribed topic. For nodes with multiple subscriptions or timers, using `rclpy.spin()` is sufficient for basic cases, but for more complex scenarios, `rclpy.executors` (e.g., `MultiThreadedExecutor`) can handle parallel callbacks.

**Advanced QoS Settings with `rclpy`:**
Configuring QoS is critical for real-world robotics. `rclpy.qos` module provides classes for this.

-   `QoSProfile`: Main class to configure QoS.
    -   `reliability`: `ReliabilityPolicy.RELIABLE` (guaranteed delivery) or `ReliabilityPolicy.BEST_EFFORT` (faster, may drop messages).
    -   `durability`: `DurabilityPolicy.VOLATILE` (only new messages to new subscribers) or `DurabilityPolicy.TRANSIENT_LOCAL` (last message to new subscribers).
    -   `history`: `HistoryPolicy.KEEP_LAST` (keep `depth` messages) or `HistoryPolicy.KEEP_ALL`.
    -   `depth`: Number of messages to keep if `KEEP_LAST` is chosen.

**Example: Python Publisher-Subscriber with Custom Message and QoS**
```python
# Create a custom message type: my_robot_msgs/msg/JointCommand.msg
# float32 joint_angle
# int32 joint_id
# string timestamp

# Then, create the ROS 2 package and build it:
# ros2 pkg create --build-type ament_python my_robot_msgs
# cd my_robot_msgs/msg
# touch JointCommand.msg (add content)
# Update setup.py and package.xml
# colcon build

# Python Publisher Node
import rclpy
from rclpy.node import Node
from my_robot_msgs.msg import JointCommand # Import custom message
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10 # Keep last 10 messages
        )
        self.publisher_ = self.create_publisher(JointCommand, 'joint_commands', qos_profile)
        self.timer = self.create_timer(0.01, self.publish_joint_command) # 100 Hz
        self.joint_id_counter = 0

    def publish_joint_command(self):
        msg = JointCommand()
        msg.joint_id = self.joint_id_counter % 6 # Cycle through 6 joints
        msg.joint_angle = 0.5 * (1 + (time.time() % 2)) # Simple oscillating angle
        msg.timestamp = str(time.time())
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: Joint {msg.joint_id}, Angle {msg.joint_angle:.2f}')
        self.joint_id_counter += 1

# Python Subscriber Node
class JointCommandSubscriber(Node):
    def __init__(self):
        super().__init__('joint_command_subscriber')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            JointCommand,
            'joint_commands',
            self.joint_command_callback,
            qos_profile)
        self.get_logger().info("JointCommandSubscriber has been initialized!")

    def joint_command_callback(self, msg: JointCommand):
        self.get_logger().info(f'Received: Joint {msg.joint_id}, Angle {msg.joint_angle:.2f} at {msg.timestamp}')

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor() # Use a multi-threaded executor
    
    publisher_node = JointCommandPublisher()
    subscriber_node = JointCommandSubscriber()
    
    executor.add_node(publisher_node)
    executor.add_node(subscriber_node)
    
    executor.spin() # Spin both nodes concurrently
    
    publisher_node.destroy_node()
    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.3. Service Client/Server in Python

Services are used for synchronous request-response communication, ideal for tasks requiring an immediate, single result.

-   **Implementing `ServiceServer` and `ServiceClient`:**
    -   `node.create_service(srv_type, service_name, callback_function)`
    -   `node.create_client(srv_type, service_name)`

**Example: Python Agent with Set Mode Service**
```python
# my_robot_msgs/srv/SetRobotMode.srv
# string mode_name
# bool enable_safety_mode
# ---
# bool success
# string message

# Python Service Server for Robot Mode
import rclpy
from rclpy.node import Node
from my_robot_msgs.srv import SetRobotMode

class RobotModeServiceServer(Node):
    def __init__(self):
        super().__init__('robot_mode_server')
        self.srv = self.create_service(SetRobotMode, 'set_robot_mode', self.handle_set_robot_mode)
        self.current_mode = "idle"
        self.safety_on = False
        self.get_logger().info('Robot mode service server ready.')

    def handle_set_robot_mode(self, request, response):
        self.current_mode = request.mode_name
        self.safety_on = request.enable_safety_mode
        self.get_logger().info(f'Set mode to: {self.current_mode}, Safety: {self.safety_on}')
        response.success = True
        response.message = f'Robot mode changed to {self.current_mode} with safety {self.safety_on}.'
        return response

# Python Service Client for Diagnostics
class DiagnosticServiceClient(Node):
    def __init__(self):
        super().__init__('diagnostic_client')
        self.client = self.create_client(SetRobotMode, 'get_robot_diagnostics') # Assuming a different service for diagnostics
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Diagnostic service not available, waiting...')
        self.req = SetRobotMode.Request() # Using same SRV type for simplicity, would be custom

    def send_diagnostic_request(self):
        self.req.mode_name = "query" # Example request
        self.req.enable_safety_mode = False # Not relevant for query
        self.future = self.client.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future) # Blocks until response
        # return self.future.result()
        return self.future # Return future to process asynchronously

# Example main for services, would need to integrate with executor if both server and client are in one process.
def main_service(args=None):
    rclpy.init(args=args)
    mode_server = RobotModeServiceServer()
    # To run client, you'd typically run it in a separate process or thread
    # client = DiagnosticServiceClient()
    # response = client.send_diagnostic_request()
    # mode_server.get_logger().info(f"Client got: {response.message}")
    rclpy.spin(mode_server)
    mode_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main_service()
```

### 2.4. Action Client/Server in Python

Actions handle long-running operations with explicit goals, continuous feedback, and a final result, perfect for navigation or complex manipulation tasks.

-   **Developing `ActionServer` and `ActionClient`:**
    -   `rclpy.action.ActionServer(node, action_type, action_name, execute_callback)`
    -   `rclpy.action.ActionClient(node, action_type, action_name)`

**Example: Python Agent with "Walk to Goal" Action**
```python
# my_robot_msgs/action/WalkToGoal.action
# float32 x
# float32 y
# float32 yaw
# ---
# bool success
# string message
# ---
# float32 distance_remaining
# float32 angle_remaining

# Python Action Server for WalkToGoal
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from my_robot_msgs.action import WalkToGoal
import time

class WalkToGoalActionServer(Node):
    def __init__(self):
        super().__init__('walk_to_goal_server')
        self._action_server = ActionServer(
            self,
            WalkToGoal,
            'walk_to_goal',
            self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback
        )
        self.get_logger().info('WalkToGoal action server ready.')

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: x={goal_request.x}, y={goal_request.y}, yaw={goal_request.yaw}')
        # Validate goal request here (e.g., check if goal is reachable)
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        # Start executing the goal in a separate thread/task
        self.get_logger().info('Goal accepted, starting execution.')
        # In a real robot, this would start a gait generator
        import threading
        thread = threading.Thread(target=self.execute_callback_thread, args=(goal_handle,))
        thread.start()

    def execute_callback_thread(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = WalkToGoal.Feedback()
        target_x, target_y = goal_handle.request.x, goal_handle.request.y
        current_x, current_y = 0.0, 0.0 # Assume robot starts at origin for simplicity

        for i in range(10): # Simulate progress
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')
                return

            current_x += target_x / 10.0
            current_y += target_y / 10.0
            feedback_msg.distance_remaining = ((target_x - current_x)**2 + (target_y - current_y)**2)**0.5
            feedback_msg.angle_remaining = goal_handle.request.yaw / 10.0 * (10 - i) # Simplified
            
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: Distance remaining {feedback_msg.distance_remaining:.2f}')
            time.sleep(0.5)

        goal_handle.succeed()
        result = WalkToGoal.Result()
        result.success = True
        result.message = 'Successfully reached goal.'
        self.get_logger().info('Goal succeeded.')
        goal_handle.set_succeeded(result)

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request.')
        return CancelResponse.ACCEPT

# Python Action Client for WalkToGoal
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from my_robot_msgs.action import WalkToGoal

class WalkToGoalActionClient(Node):
    def __init__(self):
        super().__init__('walk_to_goal_client')
        self._action_client = ActionClient(self, WalkToGoal, 'walk_to_goal')

    def send_goal(self, x, y, yaw):
        goal_msg = WalkToGoal.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.yaw = yaw

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}, Message: {result.message}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: Distance remaining {feedback_msg.feedback.distance_remaining:.2f}, Angle remaining {feedback_msg.feedback.angle_remaining:.2f}')

def main_action(args=None):
    rclpy.init(args=args)
    client = WalkToGoalActionClient()
    client.send_goal(5.0, 2.0, 0.5) # Walk 5m forward, 2m left, turn 0.5 rad
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    # You would typically run the server and client in separate processes or threads
    # For a full example, consider a launch file orchestrating both
    main_action()
```

## 3. Building Advanced Python Agents for Humanoid Control

`rclpy` enables the creation of sophisticated agents that can bridge high-level AI decisions with low-level robot control.

### 3.1. Integrating `ros2_control` with Python Agents

`ros2_control` provides a standardized hardware abstraction layer. Python agents can command humanoid joints by publishing to the appropriate `ros2_control` topics.

-   **Commanding Actuators:** Agents publish `sensor_msgs/JointState` or `trajectory_msgs/JointTrajectory` messages to topics configured by `ros2_control` controllers (e.g., `/joint_trajectory_controller/joint_trajectory`).
-   **Receiving Feedback:** Agents subscribe to `/joint_states` (published by `robot_state_publisher` or `ros2_control`'s state broadcaster) to get current joint angles, velocities, and efforts.
-   **Implementation Example (Python Agent for Simple Gait):**
    A Python agent could subscribe to a `/cmd_gait` topic (e.g., `std_msgs/String` with "walk_forward", "turn_left") and translate these into a sequence of `JointTrajectoryPoint` messages published to a `JointTrajectoryController`.

### 3.2. Sensor Data Processing with `rclpy`

Python agents are excellent for processing sensor data, especially when integrating with AI/ML libraries.

-   **Image Data:** Subscribe to `sensor_msgs/Image` topics. Use `cv_bridge` to convert ROS images to OpenCV images for processing.
    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
    import numpy as np

    class ImageProcessor(Node):
        def __init__(self):
            super().__init__('image_processor')
            self.subscription = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10)
            self.bridge = CvBridge()
            self.get_logger().info('Image processor node started.')

        def image_callback(self, msg):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().error(f'cv_bridge exception: {e}')
                return
            
            # Perform simple image processing (e.g., Canny edge detection)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray_image, 100, 200)
            
            # For visualization (requires OpenCV GUI)
            # cv2.imshow("Original Image", cv_image)
            # cv2.imshow("Edges", edges)
            # cv2.waitKey(1)
            self.get_logger().info(f'Processed image with shape: {edges.shape}')

    def main_image_processor(args=None):
        rclpy.init(args=args)
        image_processor_node = ImageProcessor()
        rclpy.spin(image_processor_node)
        image_processor_node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main_image_processor()
    ```
-   **LiDAR Point Clouds:** Subscribe to `sensor_msgs/PointCloud2`. Processing these often involves converting them to NumPy arrays or using specialized libraries for 3D data.
-   **IMU Data:** Subscribe to `sensor_msgs/Imu` for linear acceleration, angular velocity, and orientation.

### 3.3. Parameter Management

`rclpy` allows nodes to declare and retrieve parameters, enabling flexible configuration without recompiling code.

-   **Declaring Parameters:** `node.declare_parameter('param_name', default_value)`
-   **Getting Parameters:** `node.get_parameter('param_name').value`
-   **Setting Parameters:** `node.set_parameters([rclpy.Parameter('param_name', rclpy.Parameter.Type.DOUBLE, new_value)])`
-   **Dynamic Updates:** Parameters can be updated at runtime, allowing agents to adapt their behavior without restarting.

## 4. Integrating AI/ML Models into `rclpy` Agents

Python's strength in AI/ML makes `rclpy` agents ideal for integrating sophisticated intelligence.

### 4.1. Inference with Deep Learning Frameworks

Python agents can easily load and run pre-trained AI models from popular frameworks like PyTorch or TensorFlow.

-   **Object Detection Agent:**
    ```python
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D # Standard vision messages
    import torch
    import torchvision.transforms as transforms
    from torchvision.models.detection import fasterrcnn_resnet50_fpn_v2, FasterRCNN_ResNet50_FPN_V2_Weights

    class ObjectDetectionAgent(Node):
        def __init__(self):
            super().__init__('object_detection_agent')
            self.subscription = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10)
            self.detection_publisher = self.create_publisher(Detection2DArray, '/object_detections', 10)
            self.bridge = CvBridge()
            
            # Load pre-trained Faster R-CNN model
            weights = FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT
            self.model = fasterrcnn_resnet50_fpn_v2(weights=weights, box_score_thresh=0.7)
            self.model.eval()
            self.transform = weights.transforms()
            self.get_logger().info('Object Detection Agent started. Model loaded.')

        def image_callback(self, msg: Image):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                self.get_logger().error(f'cv_bridge exception: {e}')
                return
            
            # Convert OpenCV image to PIL Image for PyTorch model
            pil_image = transforms.ToPILImage()(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            
            # Preprocess image and perform inference
            with torch.no_grad():
                prediction = self.model([self.transform(pil_image)])

            detections_msg = Detection2DArray()
            detections_msg.header = msg.header
            
            for element in prediction[0]:
                score = element['score'].item()
                label = element['labels'].item()
                box = element['boxes'].int().tolist() # [xmin, ymin, xmax, ymax]

                detection = Detection2D()
                detection.header = msg.header
                detection.bbox.center.x = float((box[0] + box[2]) / 2)
                detection.bbox.center.y = float((box[1] + box[3]) / 2)
                detection.bbox.size_x = float(box[2] - box[0])
                detection.bbox.size_y = float(box[3] - box[1])
                detection.score = score
                # Add label, e.g., using COCO class names if available
                # detection.results = [ObjectHypothesisWithPose(class_id=str(label), score=score)]
                
                detections_msg.detections.append(detection)

            self.detection_publisher.publish(detections_msg)
            self.get_logger().info(f'Published {len(detections_msg.detections)} detections.')

def main_detection_agent(args=None):
    rclpy.init(args=args)
    agent = ObjectDetectionAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main_detection_agent()
    ```
### 4.2. GPU Acceleration

For real-time AI inference, leveraging NVIDIA GPUs is essential. Python agents can do this by:
-   **Direct PyTorch/TensorFlow GPU Usage:** Configuring these frameworks to use CUDA-enabled GPUs.
-   **TensorRT Integration:** For maximum performance, models can be converted to NVIDIA TensorRT format and run via Python bindings or external Isaac ROS GEMs.
-   **Isaac ROS GEMs:** As discussed in the Isaac ROS chapter, these provide pre-optimized, GPU-accelerated ROS 2 nodes that Python agents can interface with.

## 5. Advanced `rclpy` Agent Design Patterns

### 5.1. Multi-threaded Executors

For nodes that handle multiple subscriptions, services, or actions, `rclpy.executors.MultiThreadedExecutor` prevents callbacks from blocking each other, ensuring responsiveness.

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor
from my_robot_msgs.msg import JointCommand
from sensor_msgs.msg import Image
# ... import your nodes (e.g., JointCommandPublisher, ImageProcessor)

def main_multi_threaded(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    
    joint_pub_node = JointCommandPublisher()
    img_proc_node = ImageProcessor()
    
    executor.add_node(joint_pub_node)
    executor.add_node(img_proc_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        joint_pub_node.destroy_node()
        img_proc_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main_multi_threaded()
```

### 5.2. Composability

ROS 2 encourages composable nodes, where multiple nodes can run within a single process. This reduces inter-process communication overhead. `rclpy` supports this by creating nodes that can be added to a `ComposableNodeContainer`.

### 5.3. Error Handling and Robustness

-   **`try-except` blocks:** Guard against exceptions in callbacks.
-   **Logging:** Use `self.get_logger().info()`, `warn()`, `error()` for debugging and monitoring.
-   **Parameter Validation:** Validate input parameters to prevent unexpected behavior.
-   **Node Lifecycle:** Implement robust lifecycle transitions for predictable startup and shutdown.

## 6. Real-World Humanoid Examples

-   **High-Level Behavior Agent:** A Python agent utilizing `BehaviorTree.CPP` (with `py_trees_ros` bindings) to orchestrate complex humanoid behaviors like "explore environment," "find object," "grasp object." This agent would use `rclpy` to communicate with Nav2 (via actions), perception modules (via topics), and manipulation controllers (via services/actions).
-   **Perception Fusion Agent:** An `rclpy` node that subscribes to multiple sensor streams (RGB-D, LiDAR, IMU), fuses them using advanced algorithms (e.g., Kalman filters, particle filters), and provides a unified, robust state estimate of the environment and robot pose to downstream planning modules. This agent could integrate GPU-accelerated processing via external Isaac ROS GEMs.

By leveraging `rclpy`, you can create highly intelligent, flexible, and robust Python agents capable of orchestrating the complex behaviors required for autonomous humanoid robots.
