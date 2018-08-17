#include "ros_operation/ros_subscriber.h"

namespace bf_driver
{

    RosSubscriber::RosSubscriber(ros::NodeHandle &nodeHandle, vehicle::idDataMap &txMap)
                                                   : nh_(nodeHandle),
                                                     txDataMap_{txMap}
    {
        twistControlSub_ = nh_.subscribe("cmd_vel", 1, &RosSubscriber::twistControlCallback, this);


//        ROS_INFO("Subscribers launched");
    }

    RosSubscriber::~RosSubscriber()
    {

    }

    bool RosSubscriber::pushToStorage(std::uint32_t id, boost::any dt)
    {
        if (!vehicle::isIdInAllowedSet(id))
        {
            return false;
        }

        vehicle::storage tmp;

        tmp.data = dt;
        tmp.dataStatus = true;

        // проверка на существование пары в словаре
        if (txDataMap_.count(id))
        {
            // нашли
            txDataMap_[id] = tmp;
        }
        else
        {
            // отсутствует
            txDataMap_.insert(vehicle::idDataPair(id, tmp));
        }

        return true;
    }

    void RosSubscriber::twistControlCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        vehicle::twist_control tmp;

        tmp.vx = msg->linear.x / 1.0;
        tmp.vy = msg->linear.y / 1.0;
        tmp.w = msg->angular.z / 1.0;

        tmp.cntrl = 0;

        pushToStorage(vehicle::twist_control_id, tmp);
    }

}
