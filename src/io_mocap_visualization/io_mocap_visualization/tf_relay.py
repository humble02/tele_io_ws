import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFRelayNode(Node):
    def __init__(self):
        super().__init__('tf_relay_node')

        # 1. 订阅你的自定义 Topic
        self.subscription = self.create_subscription(
            TFMessage,
            '/io_fusion/tf',
            self.listener_callback,
            10
        )

        # 2. 发布到标准 TF Topic (RViz 监听这个)
        self.publisher = self.create_publisher(
            TFMessage,
            '/tf',
            10
        )

        self.get_logger().info('TF Relay Node Started: /io_fusion/tf -> /tf')

    def listener_callback(self, msg):
        # 直接转发消息
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
