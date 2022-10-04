#ifndef TRACKERS_H_
#define TRACKERS_H_

#include "ros/ros.h"
#include "gaussian_datatypes.h"
#include "dynamics_models.h"

/*
Namespace for Tracker-specific classes and data types
*/
namespace MultiObjectTrackers {

    /*
    Gaussian Mixture - Probability Hypothesis Density (GM-PHD) class
    */
    class GmPhdFilter2D
    {
        public:
            // Default constructor
            GmPhdFilter2D() {};

            // Construct with initial state at a given time
            GmPhdFilter2D(GaussianDataTypes::GaussianMixture<4> initState) : _state(initState), _lastUpdated(ros::Time::now()) {};

            // Accessors
            GaussianDataTypes::GaussianMixture<4> State() {return _state;};
            ros::Time LastUpdated() {return _lastUpdated;};

            // Mutators
            // TODO Propagate()

            // Public members
            // TODO: make this a vector if tracking multiple classes with different dynamics models
            DynamicsModels::LinearDynamics2D Dynamics;

        private:
            GaussianDataTypes::GaussianMixture<4> _state; // State estimate
            ros::Time _lastUpdated;   // ROS timestamp when state was last updated

    };

}


#endif // TRACKERS_H_