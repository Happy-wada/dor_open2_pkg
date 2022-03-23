#!/usr/bin/env python 
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 

class DoorOpen():
    def __init__(self):
        self.twist_pub = rospy.Publisher('/vmegarover/diff_drive_cintroller/cmd_vel', Twist, queue_size = 1 )
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        self.front_laser_dist = 999.9
    def laserCB(self, receive_msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def execute(self):
        vel = Twist()
        vel.linear

if __name__ == '__main__':
    rospy.init_node('door_open')
    do = DoorOpen()
    
