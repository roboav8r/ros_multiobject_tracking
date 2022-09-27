#include "multiobject_tracking_node.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "mot_node");

  ros::NodeHandle n;

  ros::Subscriber lidar_sub = n.subscribe("/legs",10, SensorCallbacks::LegTrackerCallback);

  // Create tracker object based on ROS param file
  // TODO: figure out how to pass these params into the tracker template.
  // Getting "‘n_spatial_dimensions’ is not usable in a constant expression" error
  std::string trackerType;
  int n_spatial_dimensions;
  int n_motion_states;
  n.getParam("tracker_type", trackerType);
  n.getParam("n_spatial_dimensions", n_spatial_dimensions);
  n.getParam("n_motion_states", n_motion_states);
  
  MultiObjectTrackers::GmPhdFilter<2,2> gmPhd;
  std::cout << trackerType << std::endl;
  ROS_INFO("Created %s filter with %i states \n", trackerType.c_str(), gmPhd.StateDim());


  // Create dynamics model object
  float sigmaP;
  n.getParam("sigma_process", sigmaP);
  DynamicsModels::LinearDynamics2D dynModel(0.1, sigmaP);

  ros::spin();

  return 0;
}