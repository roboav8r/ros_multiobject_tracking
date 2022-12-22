#include "multiobject_tracking_node.h"

int main(int argc, char **argv)
{
  /*
  CREATE ROS OBJECTS
  */
 
  // Create ROS node
  ros::init(argc, argv, "mot_node");
  ros::NodeHandle n;

  // Create multithreaded spinner with 2 threads
  ros::MultiThreadedSpinner multiSpinner(2);

  // Create publishers
  ros::Publisher viz_pub = n.advertise<visualization_msgs::MarkerArray>("/detection_markers",10);


  /*
  CREATE TRACKER OBJECT
  */

  // TODO: figure out how to pass these params into the tracker template.
  // Getting "‘n_spatial_dimensions’ is not usable in a constant expression" error
  // n.getParam("tracker_type", trackerType);
  // n.getParam("n_spatial_dimensions", n_spatial_dimensions);
  // n.getParam("n_motion_states", n_motion_states);

  // Read in initial state, birth model, spawn models & populate Gaussian Mixture objects
  n.getParam("x0", initialStateParams);
  n.getParam("birth_model", birthModelParams);
  n.getParam("spawn_model", spawnModelParams);
  
  // Load Dynamics Model params
  float processVar, probSurvival, spawnProcessVar, spawnProbSurvival;
  n.getParam("process_variance", processVar);
  n.getParam("p_survival", probSurvival);
  n.getParam("spawn_dynamics/process_variance", spawnProcessVar);
  n.getParam("spawn_dynamics/p_survival", spawnProbSurvival);

  // Load pruning parameters into tracker
  float truncThresh, mergeThresh;
  int maxGaussians;
  n.getParam("truncation_threshold", truncThresh);
  n.getParam("merge_threshold", mergeThresh);
  n.getParam("max_gaussians", maxGaussians);

  // Create tracker object and assign parameters to object
  GaussianDataTypes::GaussianMixture<4> initialState = ParamsToState(initialStateParams);
  GaussianDataTypes::GaussianMixture<4> birthModel = ParamsToState(birthModelParams);
  GaussianDataTypes::GaussianMixture<4> spawnModel = ParamsToState(spawnModelParams);
  //MultiObjectTrackers::GmPhdFilter2D gmPhd(initialState);
  MultiObjectTrackers::GmPhdFilter2D gmPhd(initialState, birthModel, spawnModel);
  ROS_INFO("Created 2D GM-PHD filter");
  gmPhd.Dynamics.ProcNoise(processVar);
  gmPhd.Dynamics.ProbSurvival(probSurvival);
  gmPhd.SpawnDynamics.ProcNoise(spawnProcessVar);
  gmPhd.SpawnDynamics.ProbSurvival(spawnProbSurvival);
  gmPhd.TruncThreshold(truncThresh);
  gmPhd.MergeThreshold(mergeThresh);
  gmPhd.MaxGaussians(maxGaussians);


  /*
  CREATE SENSOR OBJECTS
  */
  n.getParam("lidar_sensor", lidarParams);
  SensorModels::Position2DPersonArray personArrayModel((double)lidarParams[0]["meas_variance"], (double)lidarParams[0]["p_detection"], (double)lidarParams[0]["clutter_density"], std::string{"philbart/map"});
  ros::Subscriber leg_tracker_sub = n.subscribe<leg_tracker::PersonArray>((std::string)lidarParams[0]["topic"], 1, boost::bind(&SensorModels::Position2DPersonArray::MeasUpdateCallback, &personArrayModel, _1, &gmPhd) );

  n.getParam("vision_sensor", visionParams);
  SensorModels::Position2DSpatialDetectionArray sdArrayModel((double)visionParams[0]["meas_variance"], (double)visionParams[0]["p_detection"], (double)visionParams[0]["clutter_density"], std::string{"philbart/map"}, 1, .6);
  ros::Subscriber vision_sub = n.subscribe<depthai_ros_msgs::SpatialDetectionArray>((std::string)visionParams[0]["topic"], 1, boost::bind(&SensorModels::Position2DSpatialDetectionArray::MeasUpdateCallback, &sdArrayModel, _1, &gmPhd) );

  // // Load Sensor Model params, create model & subscriber
  // float laserMeasVar, laserProbDetect, laserClutterDens;
  // n.getParam("meas_variance", laserMeasVar);
  // n.getParam("p_detection", laserProbDetect);
  // n.getParam("clutter_density", laserClutterDens);
  // SensorModels::Position2DPoseArray laserModel(laserMeasVar, laserProbDetect, laserClutterDens, std::string{"walrus/base_link"});

  // // Create lidar subscriber
  // ros::Subscriber lidar_sub = n.subscribe<geometry_msgs::PoseArray>("/legs", 1, boost::bind(&SensorModels::Position2DPoseArray::MeasUpdateCallback, &laserModel, _1, &gmPhd) );
  // &SensorModels::Position2D::Callback, &laserModel);


  /*
  MAIN TIMER/LOOP
  */

  // Setup main propagation/visualization/publisher loop (timer)
  // ros::Rate loopRate(10);
  ros::Timer mainTimer = n.createTimer(ros::Duration(0.25),[&](const ros::TimerEvent& event){

    // // Update timestep
    // gmPhd.Dt = (event.current_real - gmPhd.LastUpdated()).toSec();

    // // Generate birth & spawn targets
    // gmPhd.PredictBirth();

    // // Predict Existing Targets
    // gmPhd.PredictState(event.current_real);

    // gmPhd.AddBirthToPredicted();
    // Predict step
    gmPhd.Predict(event.current_real);

    // Prune Gaussian mixture
    // gmPhd.Prune();

    // Publish/visualize state
    VisualizeState(viz_pub, gmPhd.State());

    // std::cout << "End Main Timer" << std::endl;
  });

  // Dummy timer to test lock/mutex
  // ros::Timer testTimer = n.createTimer(ros::Duration(0.01),[&](const ros::TimerEvent& event){
  //   std::lock_guard<std::mutex> lockg(gmPhd.TrackerMutex);
  //   std::cout << "Test Timer - simulating work in thread #"
  //             << std::this_thread::get_id() << std::endl;
  //   ros::Duration(0.05).sleep();
  //   std::cout << "Work complete" << std::endl;
  // });


  // ros::spin();
  multiSpinner.spin();

  return 0;
}