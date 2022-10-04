#ifndef TRACKERS_H_
#define TRACKERS_H_

#include <mutex>

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
            void LastUpdated(const ros::Time updateTime) {this->_lastUpdated = updateTime;}
            void TruncThreshold(const float truncThresh) {this->_truncThreshold = truncThresh;}
            void MergeThreshold(const float mergeThresh) {this->_mergeThreshold = mergeThresh;}
            void MaxGaussians(const uint maxGaussians) {this->_maxGaussians = maxGaussians;}

            // Public members
            // TODO: make this a vector if tracking multiple classes with different dynamics models
            DynamicsModels::LinearDynamics2D Dynamics;

            double dt{0.1};

            void PropagateState(const ros::Time updateTime) {
                
                // Lock resources while modifying them
                std::lock_guard<std::mutex> guard(TrackerMutex);

                dt = (updateTime - _lastUpdated).toSec();
                Dynamics.TransMatrix(dt);
                Dynamics.CovMatrix(dt);

                for (auto& gm : _state.Gaussians ) {
                    gm.Weight = Dynamics.ProbSurvival()*gm.Weight;
                    gm.Mean = Dynamics.TransMatrix()*gm.Mean;
                    gm.Cov = Dynamics.CovMatrix() + Dynamics.TransMatrix()*gm.Cov*Dynamics.TransMatrix().transpose();
                }

                _lastUpdated = updateTime;
            };

            void Prune() {
                std::lock_guard<std::mutex> guard(TrackerMutex);
                _state.prune(_truncThreshold, _mergeThreshold, _maxGaussians);
            }

            std::mutex TrackerMutex;

        private:
            GaussianDataTypes::GaussianMixture<4> _state; // State estimate
            ros::Time _lastUpdated;   // ROS timestamp when state was last updated

            // Pruning parameters
            float _truncThreshold{0.1};          
            float _mergeThreshold{4.0};
            uint _maxGaussians {10};
    };

}


#endif // TRACKERS_H_