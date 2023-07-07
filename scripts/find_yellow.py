#!/usr/bin/env python3

# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
import cv2 # OpenCV library
import numpy as np
import math

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

#globall variables
MINIMUM_YELLOW = 150 
PositionMarked = 5


 #Stops or resumes exploration 
class ExploreStop(Node): 
    def __init__(self): 
        super().__init__('Exploration_stop_publisher') 
        self.ExStop = Bool()
        self.ExStop.data = True
        self.publisher = self.create_publisher(Bool, 'explore/resume', 10)
        self.get_logger().info('Image proc explore supervisior init') 
        
    def resume(self):
        self.ExStop.data = True
        self.publisher.publish(ExStop)
        self.get_logger().info(f'Send message to explore, value:  %s' %(self.ExStop.data))
    
    def stop(self):
        self.ExStop.data = False
        self.publisher.publish(self.ExStop)
        self.get_logger().info(f'Send message to explore, value:  %s' %(self.ExStop.data))
 
 
class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('arrow_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'yellow_marker', 10)
        
        self.marker_id = 0
        self.get_logger().info('Image process init')
        
        
    def publish_marker(self, xpos, ypos):
        marker = Marker()
        
        #marker.header.frame_id = 'base_link'
        marker.header.frame_id = 'camera_point_link'
        #marker.header.frame_id = 'map'
        marker.type = Marker.CYLINDER
        
        marker.id = self.marker_id
        
        #marker.frame_locked=True
           
        marker.pose.orientation.y = 0.0
        marker.pose.position.x = xpos
        marker.pose.position.y = ypos
        marker.pose.position.z = 0.0
        marker.scale.x = 0.2  # Shaft diameter
        marker.scale.y = 0.2  # Head diameter
        marker.scale.z = 1.0  # Head length
        marker.color.r = 0.0
        marker.color.g = 1.4
        marker.color.b = 1.1
        marker.color.a = 1.7
        self.get_logger().info('Marker created')
        
        self.publisher_.publish(marker)
        self.get_logger().info(f'Marker published with ID: %d' %(marker.id))
        self.marker_id += 1 

           
        
class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        #create marker object
        self.marker_obj = MarkerPublisher()
        
        #create exploration supervisor object 
        self.explore_obj = ExploreStop() 
        
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
        
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
        Image, 
        'camera/image_raw', 
        self.listener_callback, 
        10)
        self.subscription # prevent unused variable warning
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


    def listener_callback(self, data):
        """
        Callback function.
        """
        if PositionMarked == 0: 
            self.subscription.destroy()
            
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        
        current_frame_rgb = cv2.cvtColor(current_frame, cv2.COLOR_RGB2BGR)
        self.get_yellow_object_coordinates(current_frame_rgb)
         
      
    def get_yellow_object_coordinates(self, image):
        global PositionMarked

        # Definiere den Farbbereich für Gelb im BGR Farbraum
        lower_yellow = np.array([0, 100, 100], dtype=np.uint8)
        upper_yellow = np.array([30, 255, 255], dtype=np.uint8)
        
        # Erzeuge eine Maske für den gelben Farbbereich
        yellow_mask = cv2.inRange(image, lower_yellow, upper_yellow)

        # Finde Konturen im Bild
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Sortiere die Konturen nach ihrer Fläche in absteigender Reihenfolge
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        if len(contours)>0: 
            # Extrahiere die Kontur mit der größten Fläche
            largest_contour = contours[0]
        
            # Berechne das Begrenzungsrechteck um die Kontur
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            if w > MINIMUM_YELLOW and PositionMarked > 0:
                PositionMarked -= 1
                
                # Extrahiere die Anzahl der X- und Y-Pixel
                height, width, _ = image.shape
                
                object_coordinates = x, y, x+w, y+h ,width ,height 
                yel_pos = calculate_distance_compesated(object_coordinates)
                
                fxpos = float(math.cos(yel_pos[1]) * yel_pos[0]) 
                fypos = float(math.sin(yel_pos[1]) * yel_pos[0])
                
                self.get_logger().info(f'Calculated X-Position: %d' %(fxpos))
                self.get_logger().info(f'Calculated Y-Position: %d' %(fypos))

                #stop exploration 
                self.explore_obj.stop() 
                
                #Publish marker on calculated position
                self.marker_obj.publish_marker(xpos=fxpos, ypos=fypos)



def calculate_distance_compesated(object_coordinates):
    hoizontal_fow = 80.0*math.pi/180.0
    vertical_fow = hoizontal_fow

    y=object_coordinates[1]+(object_coordinates[3]-object_coordinates[1])/2  #y plus halbe hohe
    x=object_coordinates[0]+(object_coordinates[2]-object_coordinates[0])/2  #x plus halbe breite
    
    Pixel_Nr_X          = object_coordinates[4] #800 
    Pixel_Nr_Y          = object_coordinates[5] #800 
    camera_Hight        = 0.25  # in m
    #camera_y_offset = 0.25
    Offset_Winkel_rad   = 0 #-0.78 #kammera 45° negativ gegen horizontal  
    winkel_rad = 0
    
    distance_in_px_relative_to_virtual_far_plane = (Pixel_Nr_Y/2) / math.tan((hoizontal_fow/2)) 
    
    yunterehalfte=y-(Pixel_Nr_Y/2)
      
    distance_in_px_relative_to_virtual_far_plane_for_x = (Pixel_Nr_X/2) / math.tan((vertical_fow/2)) 
    
    if(x>=(Pixel_Nr_X/2)):
        x_halfte=x-(Pixel_Nr_X/2)  #halfte der max px abziehen
    else:
        x_halfte=(Pixel_Nr_X/2)-x  #richtung invertieren
      
    x_winkel_rad=math.atan2(x_halfte , distance_in_px_relative_to_virtual_far_plane_for_x)
    
    winkel_rad=math.atan2(yunterehalfte , distance_in_px_relative_to_virtual_far_plane)
    winkel_rad= -winkel_rad
    winkel_rad=winkel_rad+Offset_Winkel_rad+math.pi/2 #+90 grad
    
    abstand = math.tan(winkel_rad) * camera_Hight

   # print("Abstand ohne seitlicher comp: {:.2f} m".format(abstand) )

    distance_in_px_relative_to_virtual_far_plane=distance_in_px_relative_to_virtual_far_plane / math.cos(abs(x_winkel_rad))  #seitliche kompensation 
    
    winkel_fur_pixel=math.atan2(yunterehalfte , distance_in_px_relative_to_virtual_far_plane)
    
    winkel_rad= -winkel_fur_pixel
    winkel_rad=winkel_rad+Offset_Winkel_rad+math.pi/2 #+90 grad
    
    abstand = math.tan(winkel_rad) * camera_Hight

    #return (abstand+camera_y_offset,x_winkel_rad)
    return (abstand, x_winkel_rad)
  

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
