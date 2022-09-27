#ifndef MULTIOBJECT_TRACKING_NODE_H_
#define MULTIOBJECT_TRACKING_NODE_H_

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>

#include "sensor_callbacks.h"
#include "trackers.h"
#include "dynamics_models.h"
#include "gaussian_datatypes.h"

// Declare variables to store ROS parameters
std::string trackerType;
int n_spatial_dimensions;
int n_motion_states;
XmlRpc::XmlRpcValue xInitial;

// Declare initial state


// Helper function to convert yaml/XmlRpc object into an initial Gaussian Mixture, x0


#endif  // MULTIOBJECT_TRACKING_NODE_H_