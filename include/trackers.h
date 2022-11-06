#ifndef TRACKERS_H_
#define TRACKERS_H_

#include <mutex>
#include <thread>

#include <ros/ros.h>

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
            GmPhdFilter2D(GaussianDataTypes::GaussianMixture<4> initState) : state_(initState), lastUpdated_(ros::Time::now()) {};

            // Construct with initial state, birth model, spawn model, and dynamics model at a given time
            GmPhdFilter2D(GaussianDataTypes::GaussianMixture<4> init_state,
            GaussianDataTypes::GaussianMixture<4> birth_model,
            GaussianDataTypes::GaussianMixture<4> spawn_model) : 
                state_(init_state),
                birthModel_(birth_model),
                spawnModel_(spawn_model),
                lastUpdated_(ros::Time::now()) {};

            // Accessors
            GaussianDataTypes::GaussianMixture<4> State() {return state_;};
            ros::Time LastUpdated() {return lastUpdated_;};

            // Mutators
            void LastUpdated(const ros::Time updateTime) {this->lastUpdated_ = updateTime;}
            void TruncThreshold(const float truncThresh) {this->truncThreshold_ = truncThresh;}
            void MergeThreshold(const float mergeThresh) {this->mergeThreshold_ = mergeThresh;}
            void MaxGaussians(const uint maxGaussians) {this->maxGaussians_ = maxGaussians;}

            // Public members
            // TODO: make these into vectors if tracking multiple classes with different dynamics models
            DynamicsModels::LinearDynamics2D Dynamics;
            DynamicsModels::LinearDynamics2D SpawnDynamics;

            std::mutex TrackerMutex;

            void PredictBirth() {
                // Reserve space for birth model and all spawned objects
                birthSpawnObjects_.Gaussians.clear();
                birthSpawnObjects_.Gaussians.reserve(birthModel_.Gaussians.size() + state_.Gaussians.size()*spawnModel_.Gaussians.size());

                // Add birth model to birthSpawnObjects_ mixture
                birthSpawnObjects_ = birthModel_;

                // Recompute spawn dynamics model
                SpawnDynamics.TransMatrix(dt_);
                SpawnDynamics.CovMatrix(dt_);

                // Add spawned objects
                for (GaussianDataTypes::GaussianModel<4> object : state_.Gaussians){
                    for (GaussianDataTypes::GaussianModel<4> spawn : spawnModel_.Gaussians) {
                        
                        GaussianDataTypes::GaussianModel<4> newSpawn;

                        newSpawn.Weight = object.Weight * spawn.Weight;
                        newSpawn.Mean = SpawnDynamics.Offset() + SpawnDynamics.TransMatrix() * object.Mean;
                        newSpawn.Cov = spawn.Cov + SpawnDynamics.TransMatrix() * object.Cov * SpawnDynamics.TransMatrix().transpose();

                        birthSpawnObjects_.Gaussians.push_back(std::move(newSpawn));

                    }
                }
            }

            void PredictState() {
                
                // Lock resources while modifying them
                std::lock_guard<std::mutex> guard(TrackerMutex);
                // std::cout << "Begin Prop in thread #" << std::this_thread::get_id() << std::endl;

                Dynamics.TransMatrix(dt_);
                Dynamics.CovMatrix(dt_);

                for (auto& gm : state_.Gaussians ) {
                    gm.Weight = Dynamics.ProbSurvival()*gm.Weight;
                    gm.Mean = Dynamics.TransMatrix()*gm.Mean;
                    gm.Cov = Dynamics.CovMatrix() + Dynamics.TransMatrix()*gm.Cov*Dynamics.TransMatrix().transpose();
                }

            };

            // Add the birth and spawn objects to the predicted objects
            void AddBirthToPredicted() {
                // TODO change state_ to predictedState_?
                state_.Gaussians.insert(state_.Gaussians.end(), 
                    birthSpawnObjects_.Gaussians.begin(), 
                    birthSpawnObjects_.Gaussians.begin() + birthSpawnObjects_.Gaussians.size());
            }

            // Predict
            void Predict(const ros::Time updateTime) {

                // Update timestep with actual value
                dt_ = (updateTime - lastUpdated_).toSec();

                // Run prediction steps
                PredictBirth();
                PredictState();
                AddBirthToPredicted();

                // Last updated time
                lastUpdated_ = updateTime;
            }

            // Update posterior weights of existing objects during the update step
            void UpdatePostWeights(const float& probDetection) {

                 // Lock resources while modifying them
                std::lock_guard<std::mutex> guard(TrackerMutex);

                for (auto& gm : state_.Gaussians ) {
                    gm.Weight = (1 - probDetection)*gm.Weight;
                }   
            }

            // During the measurement update step, add the weighted measurement objects to state
            void AddMeasurementObjects(const GaussianDataTypes::GaussianMixture<4>& meas_objects) {

                 // Lock resources while modifying them
                std::lock_guard<std::mutex> guard(TrackerMutex);

                // Reserve memory in _state
                state_.Gaussians.reserve(state_.Gaussians.size() + meas_objects.Gaussians.size());

                // add measurements to _state
                for ( auto& gm : meas_objects.Gaussians ) {
                   state_.Gaussians.emplace_back(std::move(gm)); 
                }   
            }

            void Prune() {
                std::lock_guard<std::mutex> guard(TrackerMutex);
                // std::cout << "Begin prune in thread #" << std::this_thread::get_id() << std::endl;
                state_.prune(truncThreshold_, mergeThreshold_, maxGaussians_);
                // std::cout << "End prune" << std::endl;
            }

        private:
            // Gaussian mixtures
            GaussianDataTypes::GaussianMixture<4> state_; // State estimate
            GaussianDataTypes::GaussianMixture<4> birthModel_; // Birth model
            GaussianDataTypes::GaussianMixture<4> spawnModel_; // Spawn model
            GaussianDataTypes::GaussianMixture<4> birthSpawnObjects_; // birthed and spawned objects

            // Timing
            double dt_{0.1};
            ros::Time lastUpdated_;   // ROS timestamp when state was last updated

            // Pruning parameters
            float truncThreshold_{0.1};          
            float mergeThreshold_{4.0};
            uint maxGaussians_{20};
    };

}


#endif // TRACKERS_H_