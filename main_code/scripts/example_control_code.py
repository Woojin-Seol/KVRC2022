#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 26 02:02:03 2019
@author: mason
"""

''' import libraries '''
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State

import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


''' class '''
class robot():
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.position_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.stat_sub = rospy.Subscriber('/mavros/state', State, self.stat_callback)
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(30)
        self.stat_check = False
        self.pose_check = False
        self.control_init=False
        self.stat = State()
        self.idx = 0

    def stat_callback(self, msg):
        self.stat = msg
        self.stat_check = True
    def pose_callback(self, msg):
        self.pose_check = True
        self.truth=msg.pose.position
        orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def input(self):
        if self.stat_check and self.pose_check:

            if not self.control_init:
                if not self.stat.armed:
                    self.arming(True)
                elif self.stat.mode!="OFFBOARD":
                    self.offboarding(base_mode=0, custom_mode="OFFBOARD")
                    pose_input=PoseStamped()
                    pose_input.header.stamp=rospy.Time.now()
                    pose_input.pose.orientation.w=1.0
                    self.position_pub.publish(pose_input)
                else:
                    self.control_init=True

            elif self.control_init:
                pose_input=PoseStamped()
                pose_input.header.stamp=rospy.Time.now()
                pose_input.pose.orientation.w=1.0
                if self.idx==0:
                    pose_input.pose.position.x=0.0
                    pose_input.pose.position.y=0.0
                    pose_input.pose.position.z=0.8
                    if abs(self.truth.x-0.0) < 0.2 and abs(self.truth.y-0.0) < 0.2 and abs(self.truth.z-0.8) < 0.2:
                        self.idx=1
                elif self.idx==1:
                    pose_input.pose.position.x=2.95
                    pose_input.pose.position.y=0.0
                    pose_input.pose.position.z=0.8
                    if abs(self.truth.x-2.95) < 0.2 and abs(self.truth.y-0.0) < 0.2 and abs(self.truth.z-0.8) < 0.2:
                        self.idx=2
                elif self.idx==2:
                    pose_input.pose.position.x=2.95
                    pose_input.pose.position.y=-1.6
                    pose_input.pose.position.z=0.8
                    if abs(self.truth.x-2.95) < 0.2 and abs(self.truth.y+1.6) < 0.2 and abs(self.truth.z-0.8) < 0.2:
                        self.idx=1
                self.position_pub.publish(pose_input)

##############################################################################################

mav_ctr = robot()
time.sleep(1) #wait 1 second to assure that all data comes in

''' main '''
if __name__ == '__main__':
    while 1:
        try:
            mav_ctr.input()
            mav_ctr.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
        #except:
        #    print("exception")
        #    pass
