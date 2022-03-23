#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan

class EnterRoom():
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        self.front_laser_dist = 999.9
    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]
    def execute(self):
        safe_dist = 1.0
        while not rospy.is_shutdown():
            rospy.sleep(0.3)
            print(self.front_laser_dist)
            print('!')
if __name__ == '__main__':
    rospy.init_node('enter')
    er = EnterRoom()
    er.execute()
