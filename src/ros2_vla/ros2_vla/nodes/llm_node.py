import rclpy
from rclpy.node import Node
from std_msgs.msg import String as Text # For recognized text input
from vla_msgs.srv import GetPlan       # For the GetPlan service

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        self.subscription = self.create_subscription(
            Text,
            'speech_to_text/text',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.srv = self.create_service(GetPlan, 'get_plan', self.get_plan_callback)
        self.get_logger().info('LLM Node started')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received text: "{msg.data}"')
        # Here, you would typically process the text with an LLM
        # For now, just logging

    def get_plan_callback(self, request, response):
        self.get_logger().info(f'Received GetPlan request for command: "{request.command}"')

        # Simulate LLM generating a plan based on the command
        command_lower = request.command.lower()

        if "clean the room" in command_lower:
            response.plan.goal = "clean_room"
            response.plan.steps = [
                "scan_room",
                "identify_dirty_objects",
                "navigate_to_object",
                "pick_object",
                "place_in_bin"
            ]
        elif "pick up the red box" in command_lower:
            response.plan.goal = "pick_up_object"
            response.plan.steps = [
                "locate_red_box",
                "navigate_to_red_box",
                "grasp_red_box",
                "lift_red_box"
            ]
        elif "navigate to the kitchen" in command_lower:
            response.plan.goal = "navigate_to_location"
            response.plan.steps = [
                "plan_path_to_kitchen",
                "execute_navigation_to_kitchen",
                "confirm_arrival_in_kitchen"
            ]
        else:
            response.plan.goal = f"Execute: {request.command}"
            response.plan.steps = [
                f"Step 1 for '{request.command}'",
                f"Step 2 for '{request.command}'"
            ]
        
        self.get_logger().info(f'Generated plan: {response.plan.goal}, steps: {response.plan.steps}')
        return response

def main(args=None):
    rclpy.init(args=args)
    llm_node = LLMNode()
    rclpy.spin(llm_node)
    llm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
