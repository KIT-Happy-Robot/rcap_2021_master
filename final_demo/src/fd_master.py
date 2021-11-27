#!/usr/bin/env python
# -*- coding: utf-8 -*-
#---------------------------------------------
# Title: RCAP2021 Final
# Author: Yusuke Kanazawa
# Date: 2021/11/27
# Memo:
#---------------------------------------------
import rospy
import roslib
import sys
from happymimi_navigation.srv import NaviLocation
from happymimi_msgs.srv import StrTrg
from happymimi_manipulation_msgs.srv import RecognitionToGrasping
path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, path)
from base_control import BaseControl

class FinalDemo():
    def __init__(self):
        # Service
        self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)
        self.tts_srv = rospy.ServiceProxy('/waveplay_srv', StrTrg)
        self.grasp_srv = rospy.ServiceProxy('/recognition_to_grasping', RecognitionToGrasping)
        self.arm_srv = rospy.ServiceProxy('/servo/arm', StrTrg)
        # Module
        self.bc = BaseControl()
        # Value
        self.grasp_result = False
        self.rotate = 90 #####

    def execute(self):
        rospy.loginfo('Executing: FinalDemo')
        self.tts_srv('Start Demonstration')
        rospy.sleep(0.5)
        self.navi_srv('location') #####
        rospy.sleep(0.5)
        grasp_count = 0
        self.tts_srv('Grasping an object')
        while not rospy.is_shutdown():
            rospy.sleep(0.3)
            self.grasp_result = self.grasp_srv('any').result #####
            if self.grasp_result == False:
                if grasp_count >= 3: #####
                    self.tts_srv("Sorry I couldn't grasp it.")
                    break
                else:
                    self.tts_srv("Grasp failed. I'll try again.")
                grasp_count += 1
            else:
                self.tts_srv('Successfully grasped.')
                break
        rospy.sleep(0.5)
        self.bc.rotateAngle(self.rotate)
        rospy.sleep(0.5)
        self.tts_srv('Here you are')
        self.arm_srv('give')
        self.bc.rotateAngle(-self.rotate)
        rospy.sleep(0.5)
        self.tts_srv('Finish Demonstration. Thank you very much')

if __name__ == '__main__':
    rospy.init_node('fd_master')
    fd = FinalDemo()
