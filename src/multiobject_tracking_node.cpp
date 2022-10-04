#include "multiobject_tracking_node.h"

int main(int argc, char **argv)
{
  // Create ROS node
  ros::init(argc, argv, "mot_node");
  ros::NodeHandle n;

  // Create subscribers
  ros::Subscriber lidar_sub = n.subscribe("/legs",10, SensorCallbacks::LegTrackerCallback);

  // Create publishers
  ros::Publisher viz_pub = n.advertise<visualization_msgs::MarkerArray>("/detection_markers",10);

  // Read in initial state, populate Gaussian Mixture object
  n.getParam("x0", initialStateParams);
  GaussianDataTypes::GaussianMixture<4> initialState = ParamsToState(initialStateParams);

  // Create tracker object based on ROS param file
  // TODO: figure out how to pass these params into the tracker template.
  // Getting "‘n_spatial_dimensions’ is not usable in a constant expression" error
  n.getParam("tracker_type", trackerType);
  n.getParam("n_spatial_dimensions", n_spatial_dimensions);
  n.getParam("n_motion_states", n_motion_states);
  
  MultiObjectTrackers::GmPhdFilter2D gmPhd(initialState);
  std::cout << trackerType << std::endl;
  ROS_INFO("Created 2D GM-PHD filter");


  // Create dynamics model object
  float sigmaP, pSurvival;
  n.getParam("sigma_process", sigmaP);
  n.getParam("p_survival", pSurvival);
  //DynamicsModels::LinearDynamics2D humanDynModel(0.1, sigmaP);
  gmPhd.Dynamics.ProcNoise(sigmaP);
  gmPhd.Dynamics.ProbSurvival(pSurvival);

  // Loop control
  ros::Rate loopRate(10);
  double dt{0.1};

  while (ros::ok()) {
    // Get delta T
    dt = (ros::Time::now() - gmPhd.LastUpdated()).toSec();
    std::cout << dt << std::endl;

    // Propagate step
    // TODO update dynamics model/dt
    // TODO propagate state

    // Publish/visualize state
    VisualizeState(viz_pub, gmPhd.State());
    
    // Loop control
    ros::spinOnce();
    loopRate.sleep();
  }
  return 0;
}