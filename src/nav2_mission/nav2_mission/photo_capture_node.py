#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Photo Capture Node for AS Camera
Provides a service to capture and save images from the depth camera
"""

import os
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2


class PhotoCaptureNode(Node):
    """Node that captures and saves images from camera"""
    
    def __init__(self):
        super().__init__('photo_capture_node')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/ascamera_hp60c/color/image_raw')
        self.declare_parameter('save_directory', '~/inspection_photos')
        self.declare_parameter('image_format', 'jpg')
        self.declare_parameter('image_quality', 95)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.save_directory = Path(os.path.expanduser(
            self.get_parameter('save_directory').value
        ))
        self.image_format = self.get_parameter('image_format').value
        self.image_quality = self.get_parameter('image_quality').value
        
        # Create save directory
        self.save_directory.mkdir(parents=True, exist_ok=True)
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.image_received = False
        
        # Use reentrant callback group for parallel execution
        callback_group = ReentrantCallbackGroup()
        
        # Subscribe to camera image topic
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10,
            callback_group=callback_group
        )
        
        # Create photo capture service
        self.capture_service = self.create_service(
            Trigger,
            '/ascamera_hp60c/capture',
            self.capture_callback,
            callback_group=callback_group
        )
        
        self.get_logger().info(f"Photo capture node initialized")
        self.get_logger().info(f"Image topic: {self.image_topic}")
        self.get_logger().info(f"Save directory: {self.save_directory}")
        self.get_logger().info(f"Service: /ascamera_hp60c/capture")
    
    def image_callback(self, msg: Image):
        """Store latest image from camera"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
    
    def capture_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Handle photo capture request"""
        if not self.image_received or self.latest_image is None:
            response.success = False
            response.message = "No image received from camera yet"
            self.get_logger().warn(response.message)
            return response
        
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            filename = f"inspection_{timestamp}.{self.image_format}"
            filepath = self.save_directory / filename
            
            # Save image
            if self.image_format.lower() == 'jpg':
                cv2.imwrite(str(filepath), self.latest_image, 
                          [cv2.IMWRITE_JPEG_QUALITY, self.image_quality])
            else:
                cv2.imwrite(str(filepath), self.latest_image)
            
            # Verify file was saved
            if filepath.exists():
                file_size = filepath.stat().st_size
                response.success = True
                response.message = f"Saved: {filepath} ({file_size} bytes)"
                self.get_logger().info(f"📸 Photo saved: {filename}")
            else:
                response.success = False
                response.message = f"Failed to save file: {filepath}"
                self.get_logger().error(response.message)
                
        except Exception as e:
            response.success = False
            response.message = f"Error saving image: {str(e)}"
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PhotoCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
