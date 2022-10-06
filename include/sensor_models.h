#ifndef SENSOR_MODELS_H_
#define SENSOR_MODELS_H_

#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

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
            Position2D(float variance, float prob): 
            _measVariance(variance), 
            _probDetect(prob),
            _obsMatrix(Eigen::MatrixXf::Zero(2,4)),
            _obsCovMatrix(Eigen::MatrixXf::Identity(2,2))
            {
                _obsMatrix.topLeftCorner(2,2) = Eigen::MatrixXf::Identity(2,2);
                _obsCovMatrix = _obsCovMatrix*pow(variance,2);
            };

            // Accessors
            Eigen::Matrix<float,2,4> ObsMatrix() {return _obsMatrix;}
            Eigen::Matrix<float,2,2> ObsCovMatrix() {return _obsCovMatrix;}

            // Members
            void Callback (const geometry_msgs::PoseArray::ConstPtr& msg)
            {
                ROS_INFO("LegTrackerCallback");
            }

        private:
            float _measVariance{0.1};      // Measurement noise (variance)
            float _probDetect{0.75};    // Probability of detection
            Eigen::Matrix<float, 2, 4> _obsMatrix;      // Observation matrix
            Eigen::Matrix<float, 2, 2> _obsCovMatrix;   // Observation covariance matrix
    };

}


#endif // SENSOR_MODELS_H_