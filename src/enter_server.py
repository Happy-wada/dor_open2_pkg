#!/usr/bin/env python
# -*- coding: utf-8 -*-

#----------------------------------
#Title: door_openのサービスサーバ
#Author: Shunsuke Wada
#memo:進行速度と距離を取得し、実行時間を計測する。
#memo:実行時間内でプログラムを動かす。
#----------------------------------
import rospy
#import roslib
import time
#import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from dor_open2_pkg.srv import value, valueResponse

class DoorServer():
    def __init__(self):
        #サービスサーバーの宣言
        rospy.Service('/dor_open2_server', value, self.execute)
        #パブリッシャの宣言
        self.twist_pub = rospy.Publisher('/vmegarover/diff_drive_controller/cmd_vel', Twist, queue_size = 1 )
        #サブスクライバーの宣言
        rospy.Subscriber('/scan', LaserScan, self.laserCB)
        #値の初期化
        self.front_laser_dist = 999.9

    def laserCB(self, receive_msg):
        #LIDARの視野角
        self.front_laser_dist = receive_msg.ranges[359]

    def execute(self, srv_req):
            vel = Twist()
            #サービスで距離と速度の取得
            vel.linear.x = srv_req.velocity
            target_dist = srv_req.distance
            #安全距離
            safe_dist = 1.0
            #速度と進行距離から実行時間を計測
            target_time = target_dist / vel.linear.x
            #現在の時間を格納
            start_time = time.time()
            stop_time = 0
            stop_start = 0
            stop_end = 0
            print'start door open'
            rospy.sleep(0.5)
            while not rospy.is_shutdown():
                #安全距離のときに実行
                if self.front_laser_dist >= safe_dist:
                    self.twist_pub.publish(vel)
                    
                #障害物があるとき実行
                elif self.front_laser_dist <= safe_dist:
                    stop_start = time.time()
                    print('please open door')
                    rospy.sleep(3.0)
                    stop_end = time.time() - stop_start
                    stop_time += stop_end
                    
                #時間の修正
                finish_time = time.time() - start_time - stop_time
                #実行時間に到達したとき実行
                if finish_time >= target_time:
                    print"enter_room finish [distance:", srv_req.distance, "velocity:", srv_req.velocity, "]"
                    return valueResponse(result = True)
                

if __name__ == '__main__':
    rospy.init_node('dor_open2')
    ds = DoorServer()
    rospy.spin()
