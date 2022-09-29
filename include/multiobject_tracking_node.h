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


#endif  // MULTIOBJECT_TRACKING_NODE_H_