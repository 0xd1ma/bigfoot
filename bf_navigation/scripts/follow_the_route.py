#!/usr/bin/env python

import rospy
import yaml

import sys
import math

#from take_photo import TakePhoto
from go_to_specific_point_on_map import GoToPose

from std_msgs.msg import Bool
from std_msgs.msg import UInt32
from std_msgs.msg import String

from wall_follow import WallFollow

RIGHT = 'right'
LEFT  = 'left'

old_odometry_x = 0.0
old_odometry_y = 0.0

amcl_x = 0.0
amcl_y = 0.0

distance = 0.0

ENTRY_RADIUS = 0.3

wall_session = False

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def amcl_pose_callback(msg):
    global amcl_x
    global amcl_y

    amcl_x = msg.pose.pose.position.x
    amcl_y = msg.pose.pose.position.y
    
    
def odometry_callback(msg):
    global old_odometry_x
    global old_odometry_y
    global distance
    
    new_x = msg.pose.pose.position.x
    new_y = msg.pose.pose.position.y

    if wall_session == False:
        old_odometry_x = new_x
        old_odometry_y = new_y 
        distance = 0.0
        
    if wall_session == True:
        p1 = (new_x, new_y)
        p2 = (old_odometry_x, old_odometry_y)
        distance = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

if __name__ == '__main__':

    with open("/home/robot/git/bigfoot/bf_navigation/scripts/route.yaml", 'r') as stream:
        dataMap = yaml.load(stream)
    try:
        rospy.init_node('follow_route', anonymous=False)
        
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, amcl_pose_callback)
        rospy.Subscriber("/odom", Odometry, odometry_callback)
        
        start_session_pub = rospy.Publisher('/photo/start_session', Bool, queue_size=10)
        session_state_msg = Bool()
        
        session_num_pub = rospy.Publisher('/photo/session_num', UInt32, queue_size=10)
        session_num_msg = UInt32()
        
        rack_id_pub = rospy.Publisher('/photo/rack_id', UInt32, queue_size=10)
        rack_id_msg = UInt32()

        navigator = GoToPose()
        #camera = TakePhoto()
        wall = WallFollow(LEFT)
        wall.stop()

        for obj in dataMap:

            if rospy.is_shutdown():
                break

            name = obj['filename']
            
            session_num_msg = obj['session']
            rack_id_msg = obj['rackid']

            rospy.loginfo("Go to %s pose of rack %s", name[:-4], rack_id_msg)

            if obj['rackid'] != 0:
                rospy.loginfo("Start wall follower")
                
                wall_session = True
                session_state_msg = True
                
                wall.start()
                
                start_session_pub.publish(session_state_msg)
                session_num_pub.publish(session_num_msg)
                rack_id_pub.publish(rack_id_msg)
                
                p1 = (amcl_x, amcl_y)

		dic = obj['position']
                p2 = (dic['x'], dic['y'])

                while math.hypot(p2[0] - p1[0], p2[1] - p1[1]) > ENTRY_RADIUS:
                    if rospy.is_shutdown():
                        break                    
                    p1 = (amcl_x, amcl_y)
                    rospy.loginfo("Distance to target: %s", math.hypot(p2[0] - p1[0], p2[1] - p1[1]))
                    rospy.sleep(0.1)                    
            
                #while distance < obj['distance']:
                #    if rospy.is_shutdown():
                #        break
                #    rospy.loginfo("Distance traveled: %s", distance)
                #    rospy.sleep(0.1)
                    
                rospy.loginfo("Stop wall follower")
                
                wall.stop()
                
                wall_session = False
                session_state_msg = False
                
                start_session_pub.publish(session_state_msg)
                
            else:
                rospy.loginfo("Start navi")
                success = navigator.goto(obj['position'], obj['quaternion'])

                rospy.loginfo("Stop navi")
                if not success:
                    rospy.loginfo("Failed to reach %s pose", name[:-4])
                    continue

            rospy.loginfo("Reached %s pose", name[:-4])

            # Take a photo from kinnect
            #if camera.take_picture(name):
                #rospy.loginfo("Saved image " + name)
            #else:
                #rospy.loginfo("No images received")

            rospy.sleep(1)
            
        rospy.loginfo("Mission complete")
        rospy.signal_shutdown("End")

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

