#pragma once

#ifndef ROS_PUBLISHER_H_
#define ROS_PUBLISHER_H_

#include <ros/ros.h>
#include "vehicle/vehicle_common.h"
#include <tf/transform_broadcaster.h>

namespace bf_driver
{
    using pubCallback = std::function< void(boost::any) >;

    using idPublisherMap = std::map< std::uint32_t, pubCallback >;
    using idPublisherPair = std::pair< std::uint32_t, pubCallback >;

    struct OdometryVelocity
    {
        std::double_t x;
        std::double_t y;
        std::double_t th;

        ros::Time stamp;

        OdometryVelocity()
        {
            x = 0.0;
            y = 0.0;
            th = 0.0;
        }

        void updateOdometry(const std::double_t vx, const std::double_t vy,  const std::double_t vth)
        {
            if (stamp.sec == 0 && stamp.nsec == 0)
            {
                stamp = ros::Time::now();
            }

            std::double_t dt = (ros::Time::now() - stamp).toSec();

            std::double_t delta_x = vx * dt;
            std::double_t delta_y = vy * dt;
            std::double_t delta_th = vth * dt;

//            std::cout << "dt : " << dt << "delta (x y th) : (" << delta_x << " " << delta_y << " " << delta_th << ")" << std::endl;

            x += delta_x;
            y += delta_y;
            th += delta_th;
            stamp = ros::Time::now();
        }
    };


    class RosPublisher
    {

    public:

        RosPublisher(ros::NodeHandle &nodeHandle, vehicle::idDataMap &rxMap);
        ~RosPublisher();

        // внутри осуществляется циклический просмотр словаря данных
        // дергаются коллбэки на публикацию данных в ROS
        void work(void);

    private:

        // словарь (ID, call) хранения коллбеков публикования данных в РОС
        idPublisherMap m_rosCalls;
        vehicle::idDataMap &m_rxDataMap;

        ros::NodeHandle &m_nodeHandle;

        OdometryVelocity m_odom;

        ros::Publisher m_odometryPub;

        tf::TransformBroadcaster m_odomTfBroadcaster;

        void publishOdometry(double vx, double vy, double vth);

        // ---> ADD HERE NEW PUBLISHERS
        ros::Publisher m_twistTelemetryPub;
        ros::Publisher m_imuTelemetryPub;

        // сюда добавляются коллбеки для публикации данных в РОС
        // ---> ADD HERE NEW CALLBACKS
        void publishTwistTelemetry(boost::any dt);
        void publishImuTelemetry(boost::any dt);


    };

}

#endif
