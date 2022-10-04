# ros_multiobject_tracking
This is a work in progress. I plan to implement ROS-ified GM-PHD, PMBM, and Labeled PMBM filters eventually.

# Prerequisites
Download the `.bag` files from UTBox here: https://utexas.app.box.com/folder/143135195417
and place them in the `/data` folder.

# Usage
## Development with a ROS .bag file
roslaunch ros_multiobject_tracking mot.launch dev_file:='data/1-person-3.bag' visualize:=true

## To-do list
- Add LiDAR sensor callback/measurement model
- Add pruning functions
- Add birth model
- Add spawn model
- Add clutter model



## Long-term improvements
- Lock state during propagate and update steps
- `git-lfs` for the development .rosbags
- Launch sensors and processing based on .yaml file
- add PMBM, delta-GLMB filter classes
- make tracker template / virtual class

# Credit
Much of this repo is adapted from the C++ GM-PHD implementation here: https://github.com/blefaudeux/gmphd