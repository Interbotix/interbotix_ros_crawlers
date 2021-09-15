# interbotix_xshexapod_gazebo

## Overview
This package contains the necessary config files to get any of the many Interbotix X-Series hexapods working in Gazebo. Specifically, it contains the [interbotix_texture.gazebo](config/interbotix_texture.gazebo) file which allows the black texture of the hexapods to display properly (following the method explained [here](http://answers.gazebosim.org/question/16280/how-to-use-custom-textures-on-urdf-models-in-gazebo/)). It also contains YAML files with tuned PID gains for the hexapod joints so that ros_control can control the hexapods effectively. This package is meant to be used in conjunction with a node that publishes joint positions to the appropriate topics.

## Structure
![xshexapod_gazebo_flowchart](images/xshexapod_gazebo_flowchart.png)
As shown above, the *interbotix_xshexapod_gazebo* package builds on top of the *interbotix_xshexapod_descriptions* and *gazebo_ros* packages. To get familiar with the nodes in the *interbotix_xshexapod_descriptions* package, please look at its README. The other nodes are described below:
- **gzserver** - responsible for running the physics update-loop and sensor data generation
- **gzclient** - provides a nice GUI to visualize the robot simulation
- **controller_manager** - responsible for loading and starting a set of controllers at once, as well as automatically stopping and unloading those same controllers
- **spawn_model** - adds the robot model as defined in the 'robot_description' parameter into the Gazebo world

## Usage
To run this package, type the line below in a terminal (assuming the PhantomX Mark4 is being launched).
```
$ roslaunch interbotix_xshexapod_gazebo xshexapod_gazebo.launch robot_model:=pxmark4
```
Since by default, Gazebo is started in a 'paused' state (this is done to give time for the controllers to kick in), unpause the physics once it is fully loaded by typing:
```
$ rosservice call /gazebo/unpause_physics
```
This is the bare minimum needed to get up and running. Take a look at the table below to see how to further customize with other launch file arguments.

| Argument | Description | Default Value |
| -------- | ----------- | :-----------: |
| robot_model | model type of the Interbotix Hexapod such as 'pxmark4' or 'wxmark4' | "" |
| robot_name | name of the robot (typically equal to `robot_model`, but could be anything) | "$(arg robot_model)" |
| external_urdf_loc | the file path to the custom urdf.xacro file that you would like to include in the Interbotix robot's urdf.xacro file| "" |
| use_rviz | launches Rviz | true |
| rviz_frame | fixed frame in Rviz; this should be changed to `map` or `<robot_name>/odom` if mapping or using local odometry respectively | $(arg robot_name)/base_bottom_link |
| world_name | the file path to the Gazebo 'world' file to load | refer to [xshexapod_gazebo.launch](launch/xshexapod_gazebo.launch) |
| gui | launch the Gazebo GUI | true |
| debug | Start gzserver in debug mode using gdb | false |
| paused | start Gazebo in a paused state | true |
| recording | enable Gazebo state log recording | false |
| use_sim_time | tells ROS nodes asking for time to get the Gazebo-published simulation time, published over the ROS topic /clock | true |
| use_position_controllers | set to true to have the ability to command arbitrary positions to the hexapod's joints in Gazebo | true |
