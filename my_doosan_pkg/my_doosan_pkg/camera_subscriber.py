import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.cv_bridge = CvBridge()

    def camera_callback(self, msg):
        try:
            # Convert the ROS image message to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error('Error converting ROS image message to OpenCV format: {}'.format(str(e)))
            return

        # Perform color-based object detection
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])  # Define lower and upper thresholds for red color range
        upper_red = np.array([10, 255, 255])
        lower_blue = np.array([110, 50, 50])  # Define lower and upper thresholds for blue color range
        upper_blue = np.array([130, 255, 255])
        lower_green = np.array([50, 50, 50])  # Define lower and upper thresholds for green color range
        upper_green = np.array([70, 255, 255])
        red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Process detected objects
        red_objects = self.process_objects(cv_image, red_mask, (0, 0, 255), 'Red')
        blue_objects = self.process_objects(cv_image, blue_mask, (255, 0, 0), 'Blue')
        green_objects = self.process_objects(cv_image, green_mask, (0, 255, 0), 'Green')

        # Display the processed image
        cv2.imshow('Color Detection', cv_image)
        cv2.waitKey(1)

    def process_objects(self, cv_image, mask, color, label):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Minimum area threshold to filter out noise
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, 2)
                objects.append((x, y, w, h, label))
        return objects

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
