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
            GmPhdFilter2D(GaussianDataTypes::GaussianMixture<4> initState) : lastUpdated_(ros::Time::now()) {
                stateObjects_->Gaussians = std::move(initState.Gaussians);
            };

            // Construct with initial state, birth model, spawn model, and dynamics model at a given time
            GmPhdFilter2D(GaussianDataTypes::GaussianMixture<4>& init_state,
                GaussianDataTypes::GaussianMixture<4>& birth_model,
                GaussianDataTypes::GaussianMixture<4>& spawn_model) : 
                    lastUpdated_(ros::Time::now()) 
                    {
                        
                        birthModel_.reset(new GaussianDataTypes::GaussianMixture<4>());
                        spawnModel_.reset(new GaussianDataTypes::GaussianMixture<4>());
                        birthSpawnObjects_.reset(new GaussianDataTypes::GaussianMixture<4>());
                        predictedObjects_.reset(new GaussianDataTypes::GaussianMixture<4>());
                        stateObjects_.reset(new GaussianDataTypes::GaussianMixture<4>());
                        extractedObjects_.reset(new GaussianDataTypes::GaussianMixture<4>());
                        stateObjects_->Gaussians = init_state.Gaussians;
                        birthModel_->Gaussians = birth_model.Gaussians;
                        spawnModel_->Gaussians = spawn_model.Gaussians;
                    };

            // Accessors
            GaussianDataTypes::GaussianMixture<4> State() {return stateObjects_->Gaussians;};
            ros::Time LastUpdated() {return lastUpdated_;};

            // Mutators
            void LastUpdated(const ros::Time updateTime) {this->lastUpdated_ = updateTime;}
            void TruncThreshold(const float truncThresh) {this->truncThreshold_ = truncThresh;}
            void MergeThreshold(const float mergeThresh) {this->mergeThreshold_ = mergeThresh;}
            void MaxGaussians(const uint maxGaussians) {this->maxGaussians_ = maxGaussians;}

            // Public member variables
            // TODO: make these into vectors if tracking multiple classes with different dynamics models
            DynamicsModels::LinearDynamics2D Dynamics;
            DynamicsModels::LinearDynamics2D SpawnDynamics;
            std::mutex TrackerMutex;
            bool ReadyToPredict{1};
            bool PredictComplete{0};
            bool ReadyToUpdate{0};
            bool UpdateComplete{0};


            // Public member functions
            void PredictBirth() {
                // Reserve space for birth model and all spawned objects
                birthSpawnObjects_->Gaussians.clear();
                birthSpawnObjects_->Gaussians.reserve(birthModel_->Gaussians.size() + stateObjects_->Gaussians.size()*spawnModel_->Gaussians.size());

                // Add birth model to birthSpawnObjects_ mixture
                birthSpawnObjects_->Gaussians = birthModel_->Gaussians;

                // Recompute spawn dynamics model
                SpawnDynamics.TransMatrix(dt_);
                SpawnDynamics.CovMatrix(dt_);

                // Add spawned objects
                for (GaussianDataTypes::GaussianModel<4> object : stateObjects_->Gaussians){
                    for (GaussianDataTypes::GaussianModel<4> spawn : spawnModel_->Gaussians) {
                        
                        GaussianDataTypes::GaussianModel<4> newSpawn;

                        newSpawn.Weight = object.Weight * spawn.Weight;
                        newSpawn.Mean = SpawnDynamics.Offset() + SpawnDynamics.TransMatrix() * object.Mean;
                        newSpawn.Cov = spawn.Cov + SpawnDynamics.TransMatrix() * object.Cov * SpawnDynamics.TransMatrix().transpose();

                        birthSpawnObjects_->Gaussians.push_back(std::move(newSpawn));

                    }
                }
            }

            void PredictState() {
                
                Dynamics.TransMatrix(dt_);
                Dynamics.CovMatrix(dt_);

                for (auto& gm : stateObjects_->Gaussians ) {
                    gm.Weight = Dynamics.ProbSurvival()*gm.Weight;
                    gm.Mean = Dynamics.TransMatrix()*gm.Mean;
                    gm.Cov = Dynamics.CovMatrix() + Dynamics.TransMatrix()*gm.Cov*Dynamics.TransMatrix().transpose();
                }

            };

            // Add the birth and spawn objects to the predicted objects
            void AddBirthToPredicted() {
                // TODO change stateObjects_ to predictedState_?
                stateObjects_->Gaussians.insert(stateObjects_->Gaussians.end(), 
                    birthSpawnObjects_->Gaussians.begin(), 
                    birthSpawnObjects_->Gaussians.begin() + birthSpawnObjects_->Gaussians.size());
            }

            // Predict
            void Predict(const ros::Time& updateTime) {

                std::unique_lock<std::mutex> ul(TrackerMutex,std::defer_lock);

                // Wait until the lock is available and the tracker is ready to predict
                if (ReadyToPredict) {
                    ReadyToPredict = false;
                    
                    if (ul.try_lock()) {
                        // Update timestep with actual value
                        dt_ = (updateTime - lastUpdated_).toSec();
                        std::cout << dt_ << std::endl;

                        // Run prediction steps
                        PredictBirth();
                        PredictState();
                        AddBirthToPredicted();

                        // Last updated time
                        lastUpdated_ = updateTime;

                        // Update status flags
                        ReadyToUpdate = true;
                    }   
                } 
            } // Predict

            // Update posterior weights of existing objects during the update step
            void UpdatePostWeights(const float& probDetection) {

                for (auto& gm : stateObjects_->Gaussians ) {
                    gm.Weight = (1 - probDetection)*gm.Weight;
                }   
            }

            // During the measurement update step, add the weighted measurement objects to state
            void AddMeasurementObjects(const GaussianDataTypes::GaussianMixture<4>& meas_objects) {

                // Reserve memory in _state
                stateObjects_->Gaussians.reserve(stateObjects_->Gaussians.size() + meas_objects.Gaussians.size());

                // add measurements to _state
                for ( auto& gm : meas_objects.Gaussians ) {
                   stateObjects_->Gaussians.emplace_back(std::move(gm)); 
                }   
            }

            void Prune() {
                stateObjects_->prune(truncThreshold_, mergeThreshold_, maxGaussians_);
            }

        private:
            // Gaussian mixtures
            std::unique_ptr<GaussianDataTypes::GaussianMixture<4>> birthModel_; // Birth model
            std::unique_ptr<GaussianDataTypes::GaussianMixture<4>> spawnModel_; // Spawn model
            std::unique_ptr<GaussianDataTypes::GaussianMixture<4>> birthSpawnObjects_; // birthed and spawned objects
            std::unique_ptr<GaussianDataTypes::GaussianMixture<4>> predictedObjects_; // birthed and spawned objects
            std::unique_ptr<GaussianDataTypes::GaussianMixture<4>> stateObjects_; // State estimate
            std::unique_ptr<GaussianDataTypes::GaussianMixture<4>> extractedObjects_; // State estimate

            // Timing & control
            double dt_{0.1};
            ros::Time lastUpdated_;   // ROS timestamp when state was last updated

            // Pruning parameters
            float truncThreshold_{0.1};          
            float mergeThreshold_{4.0};
            uint maxGaussians_{20};

    }; // End GMPHD class

}


#endif // TRACKERS_H_