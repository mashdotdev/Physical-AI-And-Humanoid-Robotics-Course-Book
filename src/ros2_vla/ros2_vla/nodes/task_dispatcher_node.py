import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String # For voice feedback
from vla_msgs.srv import GetPlan
from rclpy.action import ActionServer
from vla_msgs.action import ExecutePlan

class TaskDispatcherNode(Node):
    def __init__(self):
        super().__init__('task_dispatcher_node')
        self.declare_parameter('robot_command', 'none')

        # Create a client for the GetPlan service
        self.llm_service_client = self.create_client(GetPlan, 'get_plan')
        while not self.llm_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('llm_node service not available, waiting again...')
        
        self.voice_feedback_publisher = self.create_publisher(String, 'voice_feedback', 10)

        # Action server for ExecutePlan
        self._action_server = ActionServer(
            self,
            ExecutePlan,
            'execute_plan',
            self.execute_plan_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Timer to trigger getting a plan and dispatching tasks
        self.timer_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(5.0, self.timer_callback, callback_group=self.timer_group)
        self.get_logger().info('Task Dispatcher Node started')

    def execute_plan_callback(self, goal_handle):
        self.get_logger().info('Executing plan...')
        feedback_msg = ExecutePlan.Feedback()
        
        plan_goal = goal_handle.request.plan.goal
        plan_steps = goal_handle.request.plan.steps
        self.get_logger().info(f'Plan Goal: {plan_goal}')
        self.publish_voice_feedback(f"Starting to execute plan: {plan_goal}")

        for i, step in enumerate(plan_steps):
            self.get_logger().info(f'Executing step {i+1}/{len(plan_steps)}: {step}')
            feedback_msg.feedback = f"Executing step {i+1}: {step}"
            goal_handle.publish_feedback(feedback_msg)
            # Simulate work for 1 second per step
            self.get_logger().info(f'Simulating execution of: "{step}"')
            rclpy.spin_until_future_complete(self, rclpy.create_timer(1.0, lambda: None).timer_future)

        goal_handle.succeed()
        result = ExecutePlan.Result()
        result.success = True
        self.publish_voice_feedback(f"Plan '{plan_goal}' completed successfully!")
        self.get_logger().info('Plan executed successfully!')
        return result

    def timer_callback(self):
        command = self.get_parameter('robot_command').get_parameter_value().string_value
        if command == 'none':
            self.get_logger().info("No command set. Use 'ros2 param set /task_dispatcher_node robot_command \"your command\"'")
            return
        
        self.get_logger().info(f"Attempting to get plan for command: {command}")
        self.send_get_plan_request(command)

    def send_get_plan_request(self, command):
        request = GetPlan.Request()
        request.command = command
        self.get_logger().info(f"Sending request for plan for command: {command}")
        
        # Non-blocking call
        self.future = self.llm_service_client.call_async(request)
        self.future.add_done_callback(self.get_plan_response_callback)

    def get_plan_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received plan for goal: {response.plan.goal}, steps: {response.plan.steps}')
            # Here, you would typically start dispatching tasks based on the plan
            self.publish_voice_feedback(f"Plan received. Goal: {response.plan.goal}. Starting tasks.")
            
            # As soon as we get a plan, we can trigger the ExecutePlan action
            # This is a basic way to trigger, in a real scenario it would be more deliberate
            goal_msg = ExecutePlan.Goal()
            goal_msg.plan = response.plan
            
            # Create an action client to send this plan to itself (for demonstration)
            # In a real system, another node might be the action client
            self._action_client = rclpy.action.ActionClient(self, ExecutePlan, 'execute_plan')
            if not self._action_client.wait_for_server(timeout_sec=5.0):
                 self.get_logger().error('ExecutePlan action server not available after plan reception!')
                 self.publish_voice_feedback("Error: Action server not ready for plan execution.")
                 return
            self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
            self._send_goal_future.add_done_callback(self._goal_response_callback)

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.publish_voice_feedback(f"Error getting plan: {e}")

    def _feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.feedback}')

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.success}')
        self.publish_voice_feedback(f"Execution finished with success: {result.success}")

    def publish_voice_feedback(self, feedback_text):
        msg = String()
        msg.data = feedback_text
        self.voice_feedback_publisher.publish(msg)
        self.get_logger().info(f"Voice Feedback: {feedback_text}")

    def get_objects_in_room(self):
        """
        Simulates interaction with a perception stack to get a list of objects in the room.
        In a real scenario, this would involve calling a perception service or subscribing to object detection topics.
        """
        self.get_logger().info("Simulating perception: Getting objects in room...")
        # Return a dummy list of objects for simulation purposes
        return ["red box", "blue ball", "green cylinder", "cup", "table", "kitchen sink"]


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor() # Use MultiThreadedExecutor for service client callbacks
    task_dispatcher_node = TaskDispatcherNode()
    executor.add_node(task_dispatcher_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        task_dispatcher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

