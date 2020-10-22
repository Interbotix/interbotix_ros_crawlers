![crawler_banner](images/crawler_banner.png)

## Overview
![crawler_repo_structure](images/crawler_repo_structure.png)
Welcome to the *interbotix_ros_crawlers* repository! This repo contains custom ROS packages to control the various types of crawlers sold at [Interbotix](https://www.trossenrobotics.com/). These ROS packages build upon the ROS driver nodes found in the [interbotix_ros_core](https://github.com/Interbotix/interbotix_ros_core) repository. Support-level software can be found in the [interbotix_ros_toolboxes](https://github.com/Interbotix/interbotix_ros_toolboxes) repository.

## Repo Structure
```
GitHub Landing Page: Explains repository structure and contains a single directory for each type of crawler.
├── Crawler Type X Landing Page: Contains 'core' crawler ROS packages.
│   ├── Core Crawler ROS Package 1
│   ├── Core Crawler ROS Package 2
│   ├── Core Crawler ROS Package X
│   └── Examples: contains 'demo' crawler ROS packages that build upon some of the 'core' crawler ROS packages
│       ├── Demo Crawler ROS Package 1
│       ├── Demo Crawler ROS Package 2
│       ├── Demo Crawler ROS Package X
│       └── Python Scripts: contains 'demo' Python scripts that build upon modules in the interbotix_ros_toolboxes repository
│           ├── Demo Python Script 1
│           ├── Demo Python Script 2
|           └── Demo Python Script X
├── LICENSE
└── README.md
```
As shown above, there are five main levels to this repository. To clarify some of the terms above, refer to the descriptions below.

- **Crawler Type** - Any crawler that can use the same *interbotix_XXXXX_control* package is considered to be of the same type. For the most part, this division lies on the type of actuator that makes up the robot. As an example, all the X-Series hexapods are considered the same type of crawler since they all use various Dynamixel X-Series servos (despite the fact that they may come in different sizes and motor versions). However, a crawler made up of some other manufacturer's servos, or even half made up of Dynamixel servos and half made up of some other manufacturer's servos would be considered a different crawler type.

- **Core Crawler ROS Package** - This refers to 'High Profile' ROS packages that are essential to make a given crawler work. Examples of 'High Profile' ROS packages include:
    - *interbotix_XXXXX_control* - sets up the proper configurations and makes it possible to control the physical crawler
    - *interbotix_XXXXX_gazebo* - sets up the proper configurations and makes it possible to control a Gazebo simulated crawler
    - *interbotix_XXXXX_descriptions* - contains URDFs and meshes of the crawlers, making it possible to visualize them in Rviz

- **Demo Crawler ROS Package** - This refers to demo ROS packages that build upon the **Core Crawler ROS Packages**. ROS researchers could use these packages as references to learn how to develop their own ROS packages and to get a feel for how the robot works. Typical demos for a given crawler type include:
    - *interbotix_XXXXX_joy* - move a hexapod's body in-place or make it walk in the world

- **Demo Python Script** - This refers to demo Python scripts that build upon modules in the *interbotix_ros_toolboxes* repository. These modules essentially abstract away all ROS code, making it easy for a researcher with no ROS experience to interface with a crawler as if it was just another Python object. It also makes sequencing robot motion a piece of cake.

Over time, the repo will grow to include more types of crawlers.

## Contributing
Feel free to send PRs to add features to currently existing Crawler ROS packages or to include new ones. Note that all PRs should follow the structure and naming conventions outlined in the repo including documentation.

## Contributors
- [Solomon Wiznitzer](https://github.com/swiz23) - **ROS Engineer**
- [Levi Todes](https://github.com/LeTo37) - **Mechatronics & CAD Engineer**
