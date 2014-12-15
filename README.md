TrajOpt_Service
===============

AI534 Project, uses machine learning to create an initial trajectory for TrajOpt to optimize

## BUILD Instructions:

- use `catkin_make -DCMAKE_BUILD_TYPE=Release` to build from the catkin root folder which faster

more see : <http://answers.ros.org/question/71965/catkin-compiled-code-runs-3x-slower>

## to create an octomap from a .bag file using PointCloud2:

Launched "roscore" in one terminal
Edited the file "/opt/ros/groovy/stacks/octomap_mapping/octomap_server/launch/octomap_mapping.launch" mapped the cloud_in to my PointCloud2 topic and changed the name of field to "base_link" as given in the header of the bag file.
Launched octomap node: "roslaunch octomap_server octomap_mapping.launch" in a new terminal
Played the bag file: "rosbag play <filename.bag>"



### All Rights Reserved
