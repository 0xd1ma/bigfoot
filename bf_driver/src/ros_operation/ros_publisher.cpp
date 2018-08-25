#include "ros_operation/ros_publisher.h"

#include <cassert>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <boost/assign.hpp>

namespace bf_driver
{
    RosPublisher::RosPublisher(ros::NodeHandle &nodeHandle,
                               vehicle::idDataMap &rxMap) :
                               m_nodeHandle{nodeHandle},
                               m_rosCalls{},
                               m_rxDataMap{rxMap}
    {
        /***********************************************************************
         *
         * инициализируем словарь парами CAN_ID - ROS_CALLBACK
         *
        ***********************************************************************/
        m_rosCalls.insert(idPublisherPair(vehicle::twist_telemetry_id,
                pubCallback(std::bind(&RosPublisher::publishTwistTelemetry, this,
                        std::placeholders::_1))));

        m_rosCalls.insert(idPublisherPair(vehicle::imu_telemetry_id,
                pubCallback(std::bind(&RosPublisher::publishImuTelemetry, this,
                        std::placeholders::_1))));

        /***********************************************************************
         *
         * инициализируем ROS publishers
         * присваивая значения топиков
         *
        ***********************************************************************/
        m_twistTelemetryPub = m_nodeHandle.
                advertise< geometry_msgs::TwistStamped >("telemetry/twist", 10);

        m_imuTelemetryPub = m_nodeHandle.
                advertise< sensor_msgs::Imu >("telemetry/imu", 10);

        m_odometryPub = m_nodeHandle.advertise< nav_msgs::Odometry >("odom", 10);

//        ROS_INFO("ROS publisher init");

    }

    RosPublisher::~RosPublisher()
    {
        ;
    }

    void RosPublisher::work(void)
    {
        for (auto &x : m_rxDataMap)
        {
            if (x.second.dataStatus)
            {
                x.second.dataStatus = false;

                try
                {
                    // дергаем коллбеки на публикацию
                    m_rosCalls[x.first](x.second.data);
                }
                catch (const std::bad_function_call &e)
                {
                    ROS_ERROR(e.what());
                    assert(0);
                }
            }
        }
    }

    /***************************************************************************
     *
     * методы публикации данных в зависимости от ID
     *
    ***************************************************************************/
    void RosPublisher::publishTwistTelemetry(boost::any dt)
    {
        geometry_msgs::TwistStamped msg;
        vehicle::twist_telemetry tmp;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_footprint";

        try
        {
            tmp = boost::any_cast< vehicle::twist_telemetry >(dt);
        }
        catch (boost::bad_any_cast &e)
        {
            ROS_ERROR(e.what());
            assert(0);
        }

        msg.twist.linear.x = tmp.vx;
        msg.twist.linear.y = tmp.vy;
        msg.twist.linear.z = 0.0;

        msg.twist.angular.z = tmp.w;

        m_twistTelemetryPub.publish(msg);

        publishOdometry(tmp.vx, tmp.vy, tmp.w);
    }

    void RosPublisher::publishImuTelemetry(boost::any dt)
    {

    }

    void RosPublisher::publishOdometry(double vx, double vy, double vth)
    {
        m_odom.updateOdometry(vx, vy, vth);

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(m_odom.th);

        //std::cout << std::dec << "Theta: " << rad2deg(m_odom.th) << std::endl;

        //geometry_msgs::TransformStamped odom_trans;

        //odom_trans.header.stamp = ros::Time::now();
        //odom_trans.header.frame_id = "odom";
        //odom_trans.child_frame_id = "base_footprint";
        //odom_trans.transform.translation.x = m_odom.x;
        //odom_trans.transform.translation.y = m_odom.y;
        //odom_trans.transform.translation.z = 0.0;
        //odom_trans.transform.rotation = odom_quat;

        //m_odomTfBroadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;

        odom.header.frame_id = "odom";
        odom.header.stamp = ros::Time::now();

        odom.pose.pose.position.x = m_odom.x;
        odom.pose.pose.position.y = m_odom.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.pose.covariance = boost::assign::list_of(1e-1) (0) (0)  (0)  (0)  (0)
                (0) (1e-1)  (0)  (0)  (0)  (0)
                (0)   (0)  (1e6) (0)  (0)  (0)
                (0)   (0)   (0) (1e6) (0)  (0)
                (0)   (0)   (0)  (0) (1e6) (0)
                (0)   (0)   (0)  (0)  (0)  (1e-1) ;

        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;

        odom.twist.twist.angular.z = vth;

        odom.twist.covariance =  boost::assign::list_of(1e-3) (0)   (0)  (0)  (0)  (0)
                (0) (1e-3)  (0)  (0)  (0)  (0)
                (0)   (0)  (1e-3) (0)  (0)  (0)
                (0)   (0)   (0) (1e-3) (0)  (0)
                (0)   (0)   (0)  (0) (1e-3) (0)
                (0)   (0)   (0)  (0)  (0)  (1e-3) ;

        m_odometryPub.publish(odom);
    }

}
