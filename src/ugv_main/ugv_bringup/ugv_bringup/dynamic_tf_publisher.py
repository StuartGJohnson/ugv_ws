# dynamic_tf_publisher.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.025, self.broadcast)

    def broadcast(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'lidar_link'
        t.transform.translation.x = -0.028
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.164
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

rclpy.init()
node = TFBroadcaster()
rclpy.spin(node)

