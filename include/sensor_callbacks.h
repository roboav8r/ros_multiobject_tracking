#ifndef SENSOR_CALLBACKS_H_
#define SENSOR_CALLBACKS_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

namespace SensorCallbacks{

    void LegTrackerCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {

        ROS_INFO("LegTrackerCallback");

    }

}

#endif // SENSOR_CALLBACKS_H_