#!/usr/bin/env python3

# Import the necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker
import cv2
import numpy as np
import math

MINIMUM_YELLOW_PIXEL = 150
# Number >1 to ensure that markers are shown in RVIZ 
MAX_NUMBER_MARKERS = 5

 
class MarkerPublisher(Node):
    """
    Class to publish a marker on given coordinates.
    """
    def __init__(self):
        super().__init__('arrow_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'yellow_marker', 10)
        
        self.marker_id = 0
        self.get_logger().info('Image process init')
        
        
    def publish_marker(self, xpos, ypos):
        marker = Marker()
        
        # Marker will be spawned relativ to the cameras position.
        marker.header.frame_id = 'camera_point_link'
        marker.type = Marker.CYLINDER
        marker.id = self.marker_id

        marker.pose.orientation.y = 0.0
        marker.pose.position.x = xpos
        marker.pose.position.y = ypos
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2  # x-component of cylinder diameter
        marker.scale.y = 0.2  # y-component of cylinder diameter, if x=/=y it becomes an elliptical cylinder
        marker.scale.z = 1.5  # cylinder height
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.get_logger().info('Marker created')
        self.publisher_.publish(marker)
        self.get_logger().info(f'Marker published with ID: %d' %(marker.id))
        self.marker_id += 1 

           
        
class ImageSubscriber(Node):
    """
    Class to subcribe to the camera topic and process the frame.
    """
    def __init__(self):
        self.marker_obj = MarkerPublisher()
        
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 10)
        self.subscription
        self.br = CvBridge()


    def listener_callback(self, data):
        if MAX_NUMBER_MARKERS == 0: 
            self.subscription.destroy()
            
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        current_frame_rgb = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
        # Image processing
        self.getObjectCoordinates(current_frame_rgb)
         
      
    def getObjectCoordinates(self, image):
        global MAX_NUMBER_MARKERS, MINIMUM_YELLOW_PIXEL

        # Define color space (BGR encoding) and create mask for identifying the object
        lower_yellow = np.array([0, 100, 100], dtype=np.uint8)
        upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
        yellow_mask = cv2.inRange(image, lower_yellow, upper_yellow)

        # Identifying the contours given the masked image
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Identify biggest contour, sorted by size in array "contour"
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        if len(contours)>0: 
            largestContour = contours[0]
        
            # Fit rectangle around the contour
            x, y, width, height = cv2.boundingRect(largestContour)
            
            if width > MINIMUM_YELLOW_PIXEL and MAX_NUMBER_MARKERS > 0:
                MAX_NUMBER_MARKERS -= 1
                pixelHeight, pixelWidth, _ = image.shape

                # Calculate distance, angle 
                distance, angle = calcDistance(x, y, x+width, y+height ,pixelWidth ,pixelHeight)

                # Calculate x and y coordinate offsets
                xOffset = float(math.cos(angle) * distance) 
                yOffset = float(math.sin(angle) * distance)
                
                #Publish marker on calculated position
                self.marker_obj.publish_marker(xOffset, yOffset)



def calcDistance(xLowerCorner , yLowerCorner , xUpperCorner, yUpperCorner , pixelWidth , pixelHeight):
    """
    Function to calculate the distance from camera to contour centerpoint including compensation
    of curvature for a virtual camera.
    """
    horizontalFOV = 80.0*math.pi/180.0
    verticalFOV = horizontalFOV

    y = yLowerCorner + (yUpperCorner - yLowerCorner)/2
    x = xLowerCorner + (xUpperCorner - xLowerCorner)/2
    
    camera_Hight = 0.25  # in m
    cameraAngle   = 0 # in rad
    angle = 0 # in rad
    
    relPixelDistanceVirtualFarPlane = (pixelHeight/2) / math.tan((horizontalFOV/2))      
    relPixelDistanceVirtualFarPlane_xComponent = (pixelWidth/2) / math.tan((verticalFOV/2)) 

    if(x>=(pixelWidth/2)):
        xHalf=x-(pixelWidth/2)
    else:
        xHalf=(pixelWidth/2)-x

    yLowerHalf = y-(pixelHeight/2)

    # Calculate the angle of the contour middle to the optical axis
    angleToOpticalAxis=math.atan2(xHalf , relPixelDistanceVirtualFarPlane_xComponent)

    # Calculate distance from camera to contour centerpoint with curvature compensation digital camera plane
    relPixelDistanceVirtualFarPlane=relPixelDistanceVirtualFarPlane / math.cos(abs(angleToOpticalAxis))
    
    angle= -math.atan2(yLowerHalf , relPixelDistanceVirtualFarPlane) + cameraAngle + math.pi/2    
    abstand = math.tan(angle) * camera_Hight

    return (abstand, angleToOpticalAxis)
  

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)
    
    # Destroy the node explicitly
    image_subscriber.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()