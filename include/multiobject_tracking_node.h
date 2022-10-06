#ifndef MULTIOBJECT_TRACKING_NODE_H_
#define MULTIOBJECT_TRACKING_NODE_H_

#include <string>
#include <iostream>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "sensor_models.h"
#include "trackers.h"
#include "dynamics_models.h"
#include "gaussian_datatypes.h"

// Declare variables to store ROS parameters
std::string trackerType;
int n_spatial_dimensions;
int n_motion_states;
XmlRpc::XmlRpcValue initialStateParams;

// Declare initial state
GaussianDataTypes::GaussianMixture<4> initialState;

// Helper function to convert ROS yaml/XmlRpc object into an initial Gaussian Mixture, x0
GaussianDataTypes::GaussianMixture<4> ParamsToState(XmlRpc::XmlRpcValue stateParams) {

GaussianDataTypes::GaussianMixture<4> state;

    // Initialize GM component for populating the initial state belief
    GaussianDataTypes::GaussianModel<4> gmComp;

    // Populate tracker object's initial estimate with user-supplied value of x0
    for (int ii=0; ii < stateParams.size() ; ii++) {

        XmlRpc::XmlRpcValue gmTemp = stateParams[ii];

        // Assign gaussian mixture values from input file to temporary variable gmComp
        gmComp.Weight = (double)gmTemp["weight"];
        for (int jj = 0; jj < gmTemp["mu"].size(); jj++) {
            gmComp.Mean(jj) = (double)gmTemp["mu"][jj];
            gmComp.Cov(jj,jj) = (double)gmTemp["sigma"][jj];
        };

        std::cout << "Got GM component with weight = " << gmComp.Weight << ", mean = " << gmComp.Mean << ", and covariance = " << gmComp.Cov << "\n"; 

        // Add gmComp to the tracker's initial state belief vector
        state.Gaussians.push_back(gmComp);

    }

    return state;

}

// Helper function to visualize state
// TODO make all of these members of RosMotNode
void VisualizeState(const ros::Publisher& vizPub, const GaussianDataTypes::GaussianMixture<4>& currentState) {
    // Create empty MarkerArray
    visualization_msgs::MarkerArray arrayMsg = visualization_msgs::MarkerArray();

    const uint32_t positionShape = visualization_msgs::Marker::CYLINDER;
    const uint32_t velocityShape = visualization_msgs::Marker::ARROW;

    visualization_msgs::Marker posMarker;
    visualization_msgs::Marker velMarker;
    posMarker.header.frame_id = "walrus/base_link";
    posMarker.header.stamp = ros::Time::now();
    posMarker.type = positionShape;
    posMarker.action = visualization_msgs::Marker::ADD;
    velMarker.header.frame_id = "walrus/base_link";
    velMarker.header.stamp = ros::Time::now();
    velMarker.type = velocityShape;
    velMarker.points.push_back(geometry_msgs::Point());
    velMarker.points.push_back(geometry_msgs::Point());
    velMarker.action = visualization_msgs::Marker::ADD;

    
    int id=0;

    for (auto& state : currentState.Gaussians) {
        posMarker.id = id;
        posMarker.pose.position.x = state.Mean(0);
        posMarker.pose.position.y = state.Mean(1);
        posMarker.pose.position.z = 0.0;
        posMarker.scale.x = state.Cov(0,0);
        posMarker.scale.y = state.Cov(1,1);
        posMarker.scale.z = 0.1;
        posMarker.pose.orientation.x = 0.0;
        posMarker.pose.orientation.y = 0.0;
        posMarker.pose.orientation.z = 0.0;
        posMarker.pose.orientation.w = 1.0;
        posMarker.color.g = 1.0;
        posMarker.color.a = state.Weight;
        posMarker.lifetime = ros::Duration();
        arrayMsg.markers.push_back(posMarker);

        // TODO figure out how to visualize velocity covariance
        velMarker.id = id+1;
        velMarker.points[0].x = state.Mean(0);
        velMarker.points[0].y = state.Mean(1);
        velMarker.points[1].x = state.Mean(0) + state.Mean(2);
        velMarker.points[1].y = state.Mean(1) + state.Mean(3);
        velMarker.pose.position.z = 0.0;
        velMarker.scale.x = 0.1;
        velMarker.scale.y = 0.1;
        velMarker.scale.z = 0.1;
        velMarker.color.g = 1.0;
        velMarker.color.a = state.Weight;
        velMarker.lifetime = ros::Duration();
        arrayMsg.markers.push_back(velMarker);


        //arrayMsg.markers.push_back(velMarker);
        id+=2;
    }
    
    vizPub.publish(arrayMsg);

}


#endif  // MULTIOBJECT_TRACKING_NODE_H_