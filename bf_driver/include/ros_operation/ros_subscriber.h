#pragma once

#ifndef ROS_SUBSCRIBER_H_
#define ROS_SUBSCRIBER_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "vehicle/vehicle_common.h"

namespace bf_driver
{

    class RosSubscriber
    {
    public:

        RosSubscriber(ros::NodeHandle &nodeHandle, vehicle::idDataMap &txMap);
        ~RosSubscriber();

    private:

        // метод заталкивания информацию в словарь(ID, data)
        bool pushToStorage(std::uint32_t id, boost::any dt);

        ros::NodeHandle &nh_;

        vehicle::idDataMap &txDataMap_;

        /***********************************************************************
         *
         * callbacks для приема информации из пространства ROS
         *
        ***********************************************************************/
        void twistControlCallback(const geometry_msgs::Twist::ConstPtr &msg);

        ros::Subscriber twistControlSub_;

    };


}

#endif
