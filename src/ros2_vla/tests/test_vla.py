import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
import time

class TestVLA(Node):
    def __init__(self):
        super().__init__('test_vla_node')
        self.get_logger().info('Test VLA Node started')
        self.future = Future()

    def send_command_and_wait(self, command, timeout=10):
        self.get_logger().info(f"Setting robot_command parameter to: '{command}'")
        param_client = rclpy.parameter_client.AsyncParameterClient(self)
        param_client.set_parameters([Parameter('robot_command', Parameter.Type.STRING, command)])
        
        self.get_logger().info(f"Waiting for {timeout} seconds for task dispatcher to process command...")
        # In a real test, you'd subscribe to feedback topics or action results
        # For simulation, we just wait for a bit and assume success if no error is reported
        time.sleep(timeout)
        self.get_logger().info("Test complete (simulated).")
        self.future.set_result(True)


def main(args=None):
    rclpy.init(args=args)
    test_node = TestVLA()
    
    # Example commands to test
    commands_to_test = [
        "clean the room",
        "pick up the red box",
        "navigate to the kitchen",
        "do something unknown"
    ]

    for cmd in commands_to_test:
        test_node.get_logger().info(f"\n--- Running test for command: '{cmd}' ---")
        test_node.send_command_and_wait(cmd, timeout=7) # Give it some time
        # You would add assertions here in a real test framework

    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
