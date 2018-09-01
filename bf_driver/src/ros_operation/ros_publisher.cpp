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
                               m_rxDataMap{rxMap},
                               m_firstGo{true},
                               m_theta{0.0},
                               m_x_global{0.0},
                               m_y_global{0.0}
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
        vehicle::encoder_telemetry tmp;

        try
        {
            tmp = boost::any_cast< vehicle::encoder_telemetry >(dt);
        }
        catch (boost::bad_any_cast &e)
        {
            ROS_ERROR(e.what());
            assert(0);
        }

        if (m_firstGo)
        {
            m_firstGo = false;

            m_stamp = ros::Time::now();

            m_frontLeftEncoderPrevious = tmp.forward_left;
            m_frontRightEncoderPrevious = tmp.forward_right;
            m_backRightEncoderPrevious = tmp.back_right;
            m_backLeftEncoderPrevious = tmp.back_left;

            return;
        }

        double voltage = tmp.voltage_mv / 1000.0;
        
        if ( voltage < 10.0 && voltage > 9.0 )
        {
			ROS_WARN_STREAM("lower voltage: " << voltage);
		}
		else if ( voltage < 9.0 )
		{
			ROS_ERROR_STREAM("critical voltage: " << voltage);			
		}

        std::int32_t deltaForwardLeft = tmp.forward_left - m_frontLeftEncoderPrevious;
        std::int32_t deltaForwardRight = tmp.forward_right - m_frontRightEncoderPrevious;
        std::int32_t deltaBackRight = tmp.back_right - m_backRightEncoderPrevious;
        std::int32_t deltaBackLeft = tmp.back_left - m_backLeftEncoderPrevious;

        m_frontLeftEncoderPrevious = tmp.forward_left;
        m_frontRightEncoderPrevious = tmp.forward_right;
        m_backRightEncoderPrevious = tmp.back_right;
        m_backLeftEncoderPrevious = tmp.back_left;

//        std::cout << "delta forward left: " << deltaForwardLeft << std::endl;
//        std::cout << "delta forward right: " << deltaForwardRight << std::endl;
//        std::cout << "delta back right: " << deltaBackRight << std::endl;
//        std::cout << "delta back left: " << deltaBackLeft << std::endl;

        double deltaT = ros::Time::now().toSec() - m_stamp.toSec();
        m_stamp = ros::Time::now();

//        std::cout << "delta T:" << deltaT << std::endl;

        double multi = M_PI * 0.098 / 36 / 2096 / 2 / 2; // от куда /4 энкодеры ?

        double s_fl = deltaForwardLeft * multi;
        double s_fr = deltaForwardRight * multi;
        double s_br = deltaBackRight * multi;
        double s_bl = deltaBackLeft * multi;

        double sum = s_fl + s_fr + s_br + s_bl;

        double deltaX = - s_fl * sin(wheelShiftAngle) + s_bl * sin(wheelShiftAngle) + s_br * sin(wheelShiftAngle) - s_fr * sin(wheelShiftAngle);
        double deltaY = - s_fl * cos(wheelShiftAngle) - s_bl * cos(wheelShiftAngle) + s_br * cos(wheelShiftAngle) + s_fr * cos(wheelShiftAngle);

        m_x_global += ( deltaX * cos(m_theta) - deltaY * sin(m_theta) ) / 2; /// 4 / 0.18;
        m_y_global += ( deltaX * sin(m_theta) + deltaY * cos(m_theta) ) / 2; /// 4 / 0.18;
        
        m_theta += sum / 4 / 0.18;        

//        std::cout << "m_theta: " << rad2deg(m_theta) << std::endl;
//        std::cout << "X: " << m_x_global << std::endl;
//        std::cout << "Y: " << m_y_global << std::endl;

        geometry_msgs::TwistStamped msg;

        double vx = deltaX / 2 / deltaT;
        double vy = deltaY / 2 / deltaT;

        double omega = sum / 4 / 0.18 / deltaT;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "base_footprint";
        msg.twist.linear.x = vx;
        msg.twist.linear.y = vy;
        msg.twist.linear.z = 0.0;

        msg.twist.angular.z = omega;

        m_twistTelemetryPub.publish(msg);

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(m_theta);

        geometry_msgs::TransformStamped odom_trans;

        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        odom_trans.transform.translation.x = m_x_global;
        odom_trans.transform.translation.y = m_y_global;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        m_odomTfBroadcaster.sendTransform(odom_trans);

        nav_msgs::Odometry odom;

        odom.header.frame_id = "odom";
        odom.header.stamp = ros::Time::now();

        odom.pose.pose.position.x = m_x_global;
        odom.pose.pose.position.y = m_y_global;
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

        odom.twist.twist.angular.z = omega;

        odom.twist.covariance =  boost::assign::list_of(1e-1) (0)   (0)  (0)  (0)  (0)
                (0) (1e-1)  (0)  (0)  (0)  (0)
                (0)   (0)  (1e6) (0)  (0)  (0)
                (0)   (0)   (0) (1e6) (0)  (0)
                (0)   (0)   (0)  (0) (1e6) (0)
                (0)   (0)   (0)  (0)  (0)  (1e-1) ;

        m_odometryPub.publish(odom);
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
