# interbotix_xshexapod_descriptions

## Overview
This package contains the URDFs and meshes for the X-Series Interbotix Hexapods. The STL files for each robot are located in a unique folder inside the [meshes](meshes/) directory. Also in the 'meshes' directory is the [interbotix_black.png](meshes/interbotix_black.png) picture. The appearance and texture of the robots come from this picture. Next, the URDFs for the robot are located in the [urdf](urdf/) directory. They are written in 'xacro' format so that users have the ability to customize what parts of the URDF get loaded to the parameter server (see the 'Usage' section below for details). Note that all the other ROS packages in the repo reference this package to launch the robot model.

## Structure
![xshexapod_description_flowchart](images/xshexapod_description_flowchart.png)
This package contains the [xshexapod_description.launch](launch/xshexapod_description.launch) file responsible for loading parts or all of the robot model. It launches up to four nodes as described below:
- **joint_state_publisher** - responsible for parsing the 'robot_description' parameter to find all non-fixed joints and publish a JointState message with those joints defined.
- **joint_state_publisher_gui** - does the same thing as the 'joint_state_publisher' node but with a GUI that allows a user to easily manipulate the joints.
- **robot_state_publisher** - uses the URDF specified by the parameter robot_description and the joint positions from the joint_states topic to calculate the forward kinematics of the robot and publish the results via tf.
- **rviz** - displays the virtual robot model using the transforms in the 'tf' topic.

## Usage
To run this package, type the line below in a terminal. Note that the `robot_model` argument must be specified as the name of one of the URDF files located in the [urdf](/urdf) directory (excluding the '.urdf.xacro' part). For example, to launch the WidowX Mark IV hexapod, type:
```
$ roslaunch interbotix_xshexapod_descriptions xshexapod_description.launch robot_model:=wxmark4 use_joint_pub_gui:=true
```
This is the bare minimum needed to get up and running. Take a look at the table below to see how to further customize with other launch file arguments.

| Argument | Description | Default Value |
| -------- | ----------- | :-----------: |
| robot_model | model type of the Interbotix Hexapod such as 'pxmark4' or 'wxmark4' | "" |
| robot_name | name of the robot (typically equal to `robot_model`, but could be anything) | "$(arg robot_model)" |
| external_urdf_loc | the file path to the custom urdf.xacro file that you would like to include in the Interbotix robot's urdf.xacro file| "" |
| load_gazebo_configs | set this to true if Gazebo is being used; it makes sure to include Gazebo related configs in the 'robot_description' parameter so that the robot models show up black in Gazebo | false |
| use_joint_pub | launches the joint_state_publisher node | false |
| use_joint_pub_gui | launches the joint_state_publisher GUI | false |
| use_rviz | launches Rviz | true |
| rviz_frame | fixed frame in Rviz; this should be changed to `map` or `<robot_name>/odom` if mapping or using local odometry respectively | $(arg robot_name)/base_bottom_link |
| rvizconfig | file path to the config file Rviz should load | refer to [xshexapod_description.launch](launch/xshexapod_description.launch) |
| model | file path to the robot-specific URDF including arguments to be passed in | refer to [xshexapod_description.launch](launch/xshexapod_description.launch) |
