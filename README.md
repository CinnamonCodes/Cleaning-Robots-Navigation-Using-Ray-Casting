# Cleaning-Robots-Navigation-Using-Ray-Casting

## Doxygen: 

https://cinnamoncodes.github.io/Cleaning-Robots-Navigation-Using-Ray-Casting/Docs/

## To Begin these tools will be needed:
> [!IMPORTANT]
> You are required to use these to have a sucessful outcome in this project.

Follow the instructions: [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

Then follow these instructions: [NAV2](https://navigation.ros.org/getting_started/index.html)

The **Turtlebot3** is an optional step on seeing how your robot will act.

The command to install is: 
``` 
sudo apt install ros-humble-turtlebot3*
```

## Once the packages are installed we will need to clone this GitHub repository:

> [!NOTE]
> Please follow all the instructions in this GitHub first.

Create3_examples GitHub: https://github.com/iRobotEducation/create3_examples/tree/humble

## Now once all those packages are install and sourced you will run this command in your terminal:
This command will allow you to check if all the topics were found in your graph:
```
ros2 topic list
```
The graph will show all connected nodes that are currently running:
```
rqt_graph
```

We also have to set the USB permission that your lidar will be using by adding this command: 
```
ls -l /dev|grep ttyUSB
```
Then run this command which will provide you with Read/Write permissions to the serial port:
```
sudo chmod 666 /dev/ttyUSB0
```
## Making a map

To make a map you will need to run a series of commands:
> [!NOTE]
> Each command will be ran in a seperate terminal.

First launch your lidar:
```
ros2 launch create3_lidar_slam sensors_launch.py
```
This terminal will launch the SLAM toolbox:
```
ros2 launch create3_lidar_slam slam_toolbox_launch.py
```
This terminal will allow you to view your map on RViz:
```
ros2 launch create3_lidar_slam rviz_launch.py
```
Now you should be able to see your area being mapped out by the lidar, but we still have a few more commands to run to officially see the occupancy grid.

There is another helpful GitHub repository:
https://github.com/joshnewans/articubot_one.git

Also a helpful video on how to use NAV2 and ROS: https://www.youtube.com/watch?v=jkoGkAd0GYk&t=362s

## Saving and Loading the Map

To be able to save your map click on Panels adn select the SlamToolbox plugin and on the bottom left they're will be a new pane where you can save the map as a serial or regular save. We recommened using both as also stated in the video.
The map will be saved in your Home directory if not stated prior. You can make a folder for all your maps if you'd like and keep them all saved there, but rememeber to keep in mind in which directory they were stored in because you will need it to
load the map in the next command.

The command you will use to load up your used map will be:
```
ros2 launch nav2_bringup localization_launch.py map:=<your map directory><map name>.yaml use_sim_time:=false
```
> [!IMPORTANT]
> Remember to include ".yaml" it will contain the parameters of the map.
The [use_sim_time:=false] can be changed to True if using a simulation like Gazebo.

![Screenshot 2024-04-13 100558](https://github.com/CinnamonCodes/Cleaning-Robots-Navigation-Using-Ray-Casting/assets/128179393/f607e554-cc7f-4f3e-a8e0-8246278e89cc)

Now to use the NAV2 waypoints through RViz you will run:
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true 
```
This command will subscribe to the currently loaded map. At times if the map does not appear you will need to change your main map frame reliability to "Transient local".

![Screenshot 2024-05-03 233212](https://github.com/CinnamonCodes/Cleaning-Robots-Navigation-Using-Ray-Casting/assets/128179393/452ce3c9-b9bd-4d0c-92cd-51fcf1f762ce)

These would be the last 2 commands you can run. "The custome_commander.py" will be used to place autonomous waypoints within a random range based on the room coordinates. The "commander_template.py" will be the same as the autonomous script except you can create your individual waypoints.
```
ros2 run my_roomba_controller custome_commander

ros2 run commander_template.py
```
The final outcome would eventually look like this:
![linear](https://github.com/CinnamonCodes/Cleaning-Robots-Navigation-Using-Ray-Casting/assets/128179393/864fa341-6c34-4f1a-951a-d3bfe8b2a692)



