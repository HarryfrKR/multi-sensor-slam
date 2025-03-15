#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import tf2_ros
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')

        # Create static transform broadcaster
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publish static transforms
        self.publish_transforms()

    def publish_transforms(self):

        q = quaternion_from_euler(0, 0, math.pi)
        # print(q)

        # RPLIDAR C1 -> base_link
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = "base_link"
        t1.child_frame_id = "laser"
        t1.transform.translation.x = -0.009  # -9.00mm
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.05       # 500mm
        t1.transform.rotation.x = q[0]
        t1.transform.rotation.y = q[1]
        t1.transform.rotation.z = q[2]
        t1.transform.rotation.w = q[3]
        

        # RealSense D435i -> base_link
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = "base_link"
        t2.child_frame_id = "camera_link"
        t2.transform.translation.x = -0.011  # -11mm
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.040    # 400mm
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0

        # Publish transforms
        self.tf_broadcaster.sendTransform([t1, t2])
        self.get_logger().info("Published static TFs for LIDAR and RealSense camera")

def main():
    rclpy.init()
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
