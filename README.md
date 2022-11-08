# ros_multiobject_tracking
This is a work in progress. I plan to implement ROS-ified GM-PHD, PMBM, and Labeled PMBM filters eventually.

# Prerequisites
Download the `.bag` files from UTBox here: https://utexas.app.box.com/folder/143135195417
and place them in the `/data` folder.

# Usage
## Development with a ROS .bag file
roslaunch ros_multiobject_tracking mot.launch dev_file:='data/1-person-3.bag' visualize:=true

## Debugging using Valgrind
```
roslaunch ros_multiobject_tracking mot_debug_1.launch dev_file:='data/1-person-3.bag' visualize:=true
roslaunch ros_multiobject_tracking mot_debug_2.launch dev_file:='data/1-person-3.bag' visualize:=true
```

## Running tests
Change to the root directory and build the tests using catkin:
```
cd ros_multiobject_tracking
catkin run_tests ros_multiobject_tracking 
```

Reference: https://ut-ims-robotics.github.io/ros_training/html/day3/workshop_2.html


## To-do list
- Multithreading/concurrency: see 
https://levelup.gitconnected.com/ros-spinning-threading-queuing-aac9c0a793f
http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning



## Long-term improvements
- Lock state during propagate and update steps
- `git-lfs` for the development .rosbags
- Launch sensors and processing based on .yaml file
- add PMBM, delta-GLMB filter classes
- make tracker template / virtual class

# Credit
Much of this repo is adapted from the C++ GM-PHD implementation here: https://github.com/blefaudeux/gmphd