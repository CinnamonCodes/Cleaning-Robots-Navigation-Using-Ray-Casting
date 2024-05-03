#!/usr/bin/ env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from irobot_create_msgs.msg import HazardDetectionVector
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.time import Time
import time
import math
import yaml

## @note Can be changed to any .yaml file
yaml_file_path = "room_map_save.yaml"

## Open the YAML file and load its contents
with open(yaml_file_path, "r") as file:
    yaml_data = yaml.safe_load(file)

## @note This function uses the bumper to detect if it hit an object
## @param hazards 
## @returns True or False
def is_front_hazard_active(hazards):
    for detection in hazards.detections:
## hazard detecting T/F
        if (detection.type != 0):   
            return True
        return False      

class ObstacleAvoidance(Node):
    ## @param self
    def __init__(self):
        super().__init__("obstacle_avoidance_subscriber")
        self.get_logger().info("Roomba has started ")
## Twist publisher
##
##      self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)    
        self.cmd_vel_pub = self.create_publisher(   
            Twist, "/cmd_vel", 10)
## Hazard subscriber.
##
##      self.hazard_sub = self.create_subscription(HazardDetectionVector, "/hazard_detection", self.hazard_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        ## @warning QoSProfile reliabilty has to be set to BEST EFFORT. 
        self.hazard_sub = self.create_subscription( 
            HazardDetectionVector, "/hazard_detection", self.hazard_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))    
        ## @param double
        self._start_time = 0.0
        self.get_logger().info("Started Moving")

    ## @param self
    ## @param hazard_detected
    def send_velocity_command(self, hazard_detected):
        vel = Twist()
        if (hazard_detected or self._start_time + 1.5 > time.perf_counter()):
            print("Hazard Detected - Turning")
            vel.linear.x = 0.0
            ## turn right
            vel.angular.z = -math.pi / 2
        else:
            print("Moving Forward")
            ## go straight
            vel.linear.x = 0.2
            vel.angular.z = 0.0
        self.cmd_vel_pub.publish(vel)    

    ## hazard callback.
    ## @param self.
    ## @param hazards:HazardDetectionVector.
    ## @see is_front_hazard_active()
    def hazard_callback(self, hazards: HazardDetectionVector):
        if (is_front_hazard_active(hazards)):
            self._start_time = time.perf_counter()
            self.send_velocity_command(True)
        else:
            self.send_velocity_command(False)    
   
## Use spin() to continuously run the node.
def main(args=None):
    rclpy.init(args=args)
    ## run the Node
    obstacle_avoidance = ObstacleAvoidance()    
    ## spin = loop
    rclpy.spin(obstacle_avoidance)
    ## destroy the node
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()              
