#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

## navigator using x, y, z coordinates.
## Axis Z will have the robot face in a certain direction.
## @param navigator will be the Robot.
## @param position_x will be X axis.
## @param position_y will be Y axis.
## @param rotation_z will be Z axis.
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
## These are individual coordinates based on the size of a room.
##        
##      goal_pose1 = create_pose_stamped(nav, -1.99, -3.02, 0.070)
##      goal_pose2 = create_pose_stamped(nav, -2.31, -2.81, 0.0023)
##      goal_pose3 = create_pose_stamped(nav, -0.197, 0.026, 0.0044)
##
        ## Takes the previous coordinates and puts them in the array to use.
        ##
##      waypoints = [goal_pose1, goal_pose2, goal_pose3]
##      for i in range(10):
##      nav.followWaypoints(waypoints)

def main():
    # --- Init ROS2 communications and Simple Commander API ---
    rclpy.init()
    nav = BasicNavigator()

    # --- Set initial pose ---
    # !!! Comment if the initial pose is already set !!!
    #initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    #nav.setInitialPose(initial_pose)

    # --- Wait for Nav2 ---
    nav.waitUntilNav2Active()

    # --- Create some Nav2 goal poses ---
    goal_pose1 = create_pose_stamped(nav, -1.99, -3.02, 0.070)
    goal_pose2 = create_pose_stamped(nav, -2.31, -2.81, 0.0023)
    goal_pose3 = create_pose_stamped(nav, -0.197, 0.026, 0.0044)
    goal_pose4 = create_pose_stamped(nav, -0.418, 0.247, 0.0044)
    goal_pose5 = create_pose_stamped(nav, -1.90, -1.47, 0.0044)
    goal_pose6 = create_pose_stamped(nav, -2.15, -1.26, 0.0044)
    goal_pose7 = create_pose_stamped(nav, -0.796, 0.508, 0.0044)
    goal_pose8 = create_pose_stamped(nav, -1.12, 0.794, 0.0044)
    goal_pose9 = create_pose_stamped(nav, -2.02, -0.3, 0.0044)
    goal_pose10 = create_pose_stamped(nav, -1.3, 0.807, 0.0044)

    # --- Going to one pose ---
    #nav.goToPose(goal_pose1)
    #while not nav.isTaskComplete():
    #        feedback = nav.getFeedback()
    #        # print(feedback)

    # --- Follow Waypoints ---
    waypoints = [goal_pose1, goal_pose2, goal_pose3, goal_pose4, goal_pose5, goal_pose6, goal_pose7, goal_pose8, goal_pose9, goal_pose10]
    for i in range(10):
        nav.followWaypoints(waypoints)

        while not nav.isTaskComplete():
           feedback = nav.getFeedback()
            #print(feedback)

    # --- Get the result ---
    print(nav.getResult())

    # --- Shutdown ROS2 communications ---
    rclpy.shutdown()

if __name__ == '__main__':
    main()