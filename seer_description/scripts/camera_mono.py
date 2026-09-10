#!/usr/bin/env python3
"""Convert ideal Isaac RGB rendering to a monochrome sensor_msgs/Image.

This models geometric monochrome imaging, not the sensor's spectral response,
Bayer pipeline, exposure electronics or USB3 Vision transport.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class MonoCamera(Node):
    def __init__(self):
        super().__init__('mv_ch100_60um_camera')
        self.bridge= CvBridge()
        self.pub=self.create_publisher(Image,'/camera/image_raw',qos_profile_sensor_data)
        self.sub=self.create_subscription(Image,'/camera/render/image_raw',self.receive,qos_profile_sensor_data)

    def receive(self, message):
        try:
            image=self.bridge.imgmsg_to_cv2(message,'mono8')
            output=self.bridge.cv2_to_imgmsg(image,encoding='mono8')
            output.header=message.header
            self.pub.publish(output)
        except CvBridgeError as exc:
            self.get_logger().error(str(exc))


def main():
    rclpy.init();node=MonoCamera()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
