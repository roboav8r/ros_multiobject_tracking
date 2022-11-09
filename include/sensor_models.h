#ifndef SENSOR_MODELS_H_
#define SENSOR_MODELS_H_

#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
// #include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "gaussian_datatypes.h"
#include "trackers.h"

/*
Namespace for sensor model classes and data types
*/
namespace SensorModels {

    // Available sensor model types
    enum SensorTypes {laser2d};

    /*
    Base 2D position sensor (e.g. LiDAR laser rangefinder, camera detection projected onto 2D plane)
    TODO: Instead of 2D, make a general template
    */
    class Position2D
    {
        public:
            // Construct with sensor variance and detection probability
            Position2D(float variance, float prob, float clutter_density, std::string tracker_frame): 
            _measVariance(variance), 
            _probDetect(prob),
            _clutterDensity(clutter_density),
            _trackerFrame(tracker_frame),
            _obsMatrix(Eigen::MatrixXf::Zero(2,4)),
            _obsCovMatrix(Eigen::MatrixXf::Identity(2,2)),
            _listener(_tfBuffer)
            {
                _obsMatrix.topLeftCorner(2,2) = Eigen::MatrixXf::Identity(2,2);
                _obsCovMatrix = _obsCovMatrix*pow(variance,2);
            };

            // Accessors
            Eigen::Matrix<float,2,4> ObsMatrix() {return _obsMatrix;}
            Eigen::Matrix<float,2,2> ObsCovMatrix() {return _obsCovMatrix;}

            // Members
            void MeasUpdateCallback (const geometry_msgs::PoseArray::ConstPtr& msg, MultiObjectTrackers::GmPhdFilter2D* tracker)
            {
                // Store/process incoming data
                // TODO: lock the tracker object here when reading state?
                std::cout << "Measurement Callback"<<std::endl;
                _expectedStates = tracker->State();

                // Create update components
                _nObjects = _expectedStates.Gaussians.size();
                std::cout << "Expecting " << _nObjects << " objects" << std::endl; 
                _expectedMeas.clear();
                _expectedMeas.reserve(_nObjects);
                _innovCovMatrix.clear();
                _innovCovMatrix.reserve(_nObjects);
                _kalmanGain.clear();
                _kalmanGain.reserve(_nObjects);
                _postCovMatrix.clear();
                _postCovMatrix.reserve(_nObjects);

                std::cout << "Computing update components" << std::endl;

                // Compute components of expected state Gaussians
                for (auto const &obj : _expectedStates.Gaussians)
                {
                    _expectedMeas.push_back(_obsMatrix*obj.Mean);
                    _innovCovMatrix.push_back(_obsCovMatrix + _obsMatrix*obj.Cov*_obsMatrix.transpose());
                    _kalmanGain.push_back(obj.Cov*_obsMatrix.transpose()*_innovCovMatrix.back().inverse());
                    _postCovMatrix.push_back((Eigen::Matrix<float, 4, 4>::Identity() - _kalmanGain.back()*_obsMatrix)*obj.Cov);
                }

                std::cout << "Updating weights of expected targets"<<std::endl;
                // Update weights of existing objects based on detection probability
                tracker->UpdatePostWeights(_probDetect);

                std::cout << "Computing measurement matches" <<std::endl;
                // Iterate through all measurements, compute measurement/state association probabilities
                geometry_msgs::PoseStamped originalPose;
                originalPose.header = msg->header;
                _nDetections = msg->poses.size();

                for (auto& pose : msg->poses ) // Iterate through detections
                {
                    // Clear and reserve memory
                    _newStates.Gaussians.clear(); // Clear new state associations
                    _newStates.Gaussians.reserve(_nObjects); // Reserve memory for each measurement/state match

                    originalPose.pose = pose;
                    geometry_msgs::PoseStamped transformedPose;

                    // Convert sensor measurement into tracker frame TODO: move this to beginning of callback or initial
                    try {
                        _tfBuffer.transform(originalPose,transformedPose,_trackerFrame);
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN("%s",ex.what());
                        ros::Duration(1.0).sleep();
                        continue;
                    }

                    _transformedMeasurement(0) = transformedPose.pose.position.x;
                    _transformedMeasurement(1) = transformedPose.pose.position.y;

                    // Compute new measurement association/match with known objects
                    float matchWeightSum{0};
                    for (uint nObject = 0; nObject < _nObjects; ++nObject)
                    {
                        GaussianDataTypes::GaussianModel<4> matchObject;
                        matchObject.Weight = _probDetect*_expectedStates.Gaussians[nObject].Weight*GaussianDataTypes::MultiVarGaussPdf<2>(_transformedMeasurement, _expectedMeas[nObject], _innovCovMatrix[nObject]);
                        matchObject.Mean = _expectedStates.Gaussians[nObject].Mean + _kalmanGain[nObject]*(_transformedMeasurement - _expectedMeas[nObject]);
                        matchObject.Cov = _postCovMatrix[nObject];

                        _newStates.Gaussians.emplace_back(std::move(matchObject));

                        matchWeightSum += matchObject.Weight; // Compute running sum for normalization
                    }

                    // Normalize the new, matched objects' weights
                    for (GaussianDataTypes::GaussianModel<4> match : _newStates.Gaussians)
                    {
                        match.Weight/=(matchWeightSum + _clutterDensity);
                    }

                    // Add matches to the tracker's state
                    tracker->AddMeasurementObjects(_newStates);

                } // measurement for loop

                // Prune 
                std::cout << "Pruning" <<std::endl;
                tracker->Prune();

            } // Measurement update callback

        private:
            float _measVariance{0.1};      // Measurement noise (variance)
            float _probDetect{0.75};    // Probability of detection
            float _clutterDensity{1.0};  // Expected number of false detections for this sensor

            Eigen::Matrix<float, 2, 4> _obsMatrix;      // Observation matrix
            Eigen::Matrix<float, 2, 2> _obsCovMatrix;   // Observation covariance matrix

            GaussianDataTypes::GaussianMixture<4> _expectedStates;       // Expected/existing states from tracker
            GaussianDataTypes::GaussianMixture<4> _newStates;            // New states from measurement

            int _nObjects;  // number of objects in state
            int _nDetections; // Number of detections in latest measurement
            std::vector<Eigen::Matrix<float, 2, 1>> _expectedMeas;      // Expected measurement mean
            std::vector<Eigen::Matrix<float, 2, 2>> _innovCovMatrix;    // Innovation covariance matrix
            std::vector<Eigen::Matrix<float, 4, 2>> _kalmanGain;        // Kalman gain matrices
            std::vector<Eigen::Matrix<float, 4, 4>> _postCovMatrix;     // Posterior state covariance matrix

            Eigen::Matrix<float, 2, 1> _transformedMeasurement;        // Measurement converted into the tracker frame

            tf2_ros::Buffer _tfBuffer;                  // Transform buffer
            tf2_ros::TransformListener _listener; // Transform listener
            geometry_msgs::TransformStamped _transform;            // transform between sensor and tracker frames
            std::string _trackerFrame;
    
    }; // Position2D class

} // SensorModels namespace

#endif // SENSOR_MODELS_H_