#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg
import struct

class LivoxCustomToPointCloud2(Node):
    def __init__(self):
        super().__init__('livox_custom_to_pointcloud2')
        self.subscription = self.create_subscription(
            CustomMsg,
            '/livox/lidar',
            self.custom_callback,
            10)
        self.publisher = self.create_publisher(
            PointCloud2,
            '/livox/pointcloud2',
            10)
        self.get_logger().info('Livox CustomMsg to PointCloud2 converter started')
        self.count = 0
        
    def custom_callback(self, msg):
        if len(msg.points) == 0:
            return
            
        cloud_msg = PointCloud2()
        cloud_msg.header = msg.header
        cloud_msg.header.frame_id = 'livox_frame'
        cloud_msg.height = 1
        cloud_msg.width = len(msg.points)
        cloud_msg.is_dense = False
        cloud_msg.is_bigendian = False
        
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        
        points_data = []
        for p in msg.points:
            points_data.extend([
                struct.pack('f', p.x),
                struct.pack('f', p.y),
                struct.pack('f', p.z),
                struct.pack('f', p.reflectivity)
            ])
        cloud_msg.data = b''.join(points_data)
        
        self.publisher.publish(cloud_msg)
        self.count += 1
        if self.count % 100 == 0:
            self.get_logger().info(f'Published {len(msg.points)} points')

def main(args=None):
    rclpy.init(args=args)
    node = LivoxCustomToPointCloud2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
