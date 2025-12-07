import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Placeholder for audio messages
from std_msgs.msg import String as Text # Placeholder for text messages

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.subscription = self.create_subscription(
            String,
            'audio_capture/audio',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Text, 'speech_to_text/text', 10)
        self.get_logger().info('Whisper Node started')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
        # Simulate Whisper ASR service
        recognized_text = f"Recognized: {msg.data}" # Replace with actual Whisper integration

        text_msg = Text()
        text_msg.data = recognized_text
        self.publisher_.publish(text_msg)
        self.get_logger().info(f'Publishing text: "{text_msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperNode()
    rclpy.spin(whisper_node)
    whisper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
