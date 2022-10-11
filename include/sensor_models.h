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
            Position2D(float variance, float prob, std::string tracker_frame): 
            _measVariance(variance), 
            _probDetect(prob),
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
                ROS_INFO("Position 2D MeasUpdateCallback");
                ROS_INFO("msg: ");
                std::cout << (*msg) << std::endl;
                // ROS_INFO("Tracker: ");
                // std::cout << tracker.Gaussians() << std::endl;

                // Store/process incoming data
                _expectedState = tracker->State();

                // Create update components
                _nObjects = _expectedState.Gaussians.size();
                _expectedMeas.clear();
                _expectedMeas.reserve(_nObjects);
                _innovCovMatrix.clear();
                _innovCovMatrix.reserve(_nObjects);
                _kalmanGain.clear();
                _kalmanGain.reserve(_nObjects);
                _postCovMatrix.clear();
                _postCovMatrix.reserve(_nObjects);

                for (auto const &obj : _expectedState.Gaussians)
                {
                    _expectedMeas.push_back(_obsMatrix*obj.Mean);
                    _innovCovMatrix.push_back(_obsCovMatrix + _obsMatrix*obj.Cov*_obsMatrix.transpose());
                    _kalmanGain.push_back(obj.Cov*_obsMatrix.transpose()*_innovCovMatrix.back().inverse());
                    _postCovMatrix.push_back((Eigen::Matrix<float, 4, 4>::Identity() - _kalmanGain.back()*_obsMatrix)*obj.Cov);
                }

                // Update weights of existing objects based on detection probability
                tracker->UpdatePostWeights(_probDetect);

                // TODO reserve additional memory in tracker state for matched targets

                // Iterate through all measurements
                geometry_msgs::PoseStamped originalPose;
                originalPose.header = msg->header;
                for (auto& pose : msg->poses )
                {
                    // std::cout << "Got measurement" << pose << std::endl;
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
                    // std::cout << "Transformed" << transformedPose << std::endl;

                    _transformedMeasurement(0) = transformedPose.pose.position.x;
                    _transformedMeasurement(1) = transformedPose.pose.position.y;
                    std::cout << "Transformed \n" << _transformedMeasurement << std::endl;

                    // TODO - iterate through each expected state & add to state

                }

                // Match measurements to expected targets

            }

        private:
            float _measVariance{0.1};      // Measurement noise (variance)
            float _probDetect{0.75};    // Probability of detection

            Eigen::Matrix<float, 2, 4> _obsMatrix;      // Observation matrix
            Eigen::Matrix<float, 2, 2> _obsCovMatrix;   // Observation covariance matrix

            GaussianDataTypes::GaussianMixture<4> _expectedState;       // Expected/previous state

            int _nObjects;  // number of objects in state
            std::vector<Eigen::Matrix<float, 2, 1>> _expectedMeas;      // Expected measurement mean
            std::vector<Eigen::Matrix<float, 2, 2>> _innovCovMatrix;    // Innovation covariance matrix
            std::vector<Eigen::Matrix<float, 4, 2>> _kalmanGain;        // Kalman gain matrices
            std::vector<Eigen::Matrix<float, 4, 4>> _postCovMatrix;     // Posterior state covariance matrix

            Eigen::Matrix<float, 2, 1> _transformedMeasurement;        // Measurement converted into the tracker frame

            tf2_ros::Buffer _tfBuffer;                  // Transform buffer
            tf2_ros::TransformListener _listener; // Transform listener
            geometry_msgs::TransformStamped _transform;            // transform between sensor and tracker frames
            std::string _trackerFrame;
    };

}


#endif // SENSOR_MODELS_H_