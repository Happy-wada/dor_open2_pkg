#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy 
import roslib 
import sys
from sensor_msgs.msg import LaserScan
from dor_open2_pkg.srv import value, valueResponse
file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, file_path)
from base_control import BaseControl

class DoorOpen():
    def __init__(self):
        rospy.Service('/door_open_server', value, self.execute)
        rospy.loginfo('start door open ')
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        self.front_laser_dist = 999.9

        self.base_control = BaseControl()

        

    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]
    def execute(self, srv_req):
        safe_dist = 2.0
        #distance = 2.0
        #velocity = 0.2
        print(self.front_laser_dist)
        rospy.sleep(0.3)
        while not rospy.is_shutdown():
            if self.front_laser_dist >= safe_dist:
                rospy.sleep(0.3)
                print(self.front_laser_dist)
                rospy.loginfo('start forward')
                rospy.sleep(0.1)
                for i in range(1):
                    self.base_control.translateDist(srv_req.distance, srv_req.velocity)
                    print('finish open the door')
                    return valueResponse(result = True)
            elif self.front_laser_dist <= safe_dist:
                rospy.sleep(0.3)
                print(self.front_laser_dist)
                print('please open the door')
                rospy.sleep(1.0)
if __name__ == '__main__':
    rospy.init_node('door2_open')
    do = DoorOpen()
    #do.execute()
    rospy.spin()
        


