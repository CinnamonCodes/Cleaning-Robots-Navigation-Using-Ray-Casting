#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import random

## navigator using x, y, z coordinates.
## Axis Z will have the robot face in a certain direction.
## @param navigator will be the Robot.
## @param position_x will be X axis.
## @param position_y will be Y axis.
## @param rotation_z will be Z axis.
##
##      def create_pose_stamped(navigator, position_x, position_y, rotation_z):
## @return goal_pose.
def create_pose_stamped(navigator, position_x, position_y, rotation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose

## random waypoint generator within a given range.
## @see create_pose_stamped()
##
##      position_x = random.uniform(-2.0, 2.0)
## @returns position_x.
## position_y.
## position_z.
def generate_random_waypoint():
    ## Generate random x and y coordinates within a range.
    position_x = random.uniform(-2.0, 2.0)
    position_y = random.uniform(-2.0, 2.0)
    ## Generate a random orientation
    rotation_z = random.uniform(0, 2.0 * 3.14159)
    return position_x, position_y, rotation_z

## Size of the array.
        ##
        ##
##      num_waypoints = 30.
##      waypoints = [].
##
##      for _ in range(num_waypoints):.
##
        ## calls random_waypoint function.
        ##
##        waypoint = generate_random_waypoint().
##
        ## adds the waypoint to create_pose_stamped function.
        ##
##        waypoints.append(create_pose_stamped(nav, *waypoint)).


def main():
    ## --- Init ROS2 communications and Simple Commander API ---
    rclpy.init()
    nav = BasicNavigator()

    ## --- Set initial pose ---
    ## !!! Comment if the initial pose is already set !!!
    ##initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    ## nav.setInitialPose(initial_pose)

    ## --- Wait for Nav2 ---
    nav.waitUntilNav2Active()

    ## --- Create waypoints ---
    ## num_waypoints is set to 30 this int can be changed it represents how large the array will be
    num_waypoints = 30
    waypoints = []
    for _ in range(num_waypoints):
        ## calls random_waypoint function
        waypoint = generate_random_waypoint()
        ## adds the waypoint to create_pose_stamped function
        waypoints.append(create_pose_stamped(nav, *waypoint))

    ## --- Go to each waypoint ---
    for waypoint in waypoints:
        nav.goToPose(waypoint)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            # You can print or process feedback here if needed

        while not nav.isTaskComplete():
           feedback = nav.getFeedback()
    #         # print(feedback)    

    # --- Get the result ---
    print(nav.getResult())

    # --- Shutdown ROS2 communications ---
    rclpy.shutdown()

if __name__ == '__main__':
    main()
