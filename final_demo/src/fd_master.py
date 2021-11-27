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
from happymimi_manipulation_msgs.srv import RecognitionToGrasping, RecognitionToGraspingResponse
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
        self.execute()

    def execute(self):
        rospy.loginfo('Executing: FinalDemo')
        self.tts_srv('/fd/start_fd')
        rospy.sleep(0.5)
        self.navi_srv('Tall table') #####
        rospy.sleep(0.5)
        grasp_count = 0
        self.tts_srv('/fd/grasp_start')
        while not rospy.is_shutdown():
            rospy.sleep(0.3)
            self.grasp_result = self.grasp_srv(RecognitionToGraspingResponse('any')).result #####
            if self.grasp_result == False:
                if grasp_count >= 3: #####
                    self.tts_srv("/fd/grasp_failed")
                    break
                else:
                    self.tts_srv("/fd/grasp_retry")
                grasp_count += 1
            else:
                self.tts_srv('/fd/grasp_success')
                break
        rospy.sleep(0.5)
        #self.bc.rotateAngle(self.rotate)
        self.navi_srv('operator')
        rospy.sleep(0.5)
        self.tts_srv('/fd/give_object')
        self.arm_srv('give')
        rospy.sleep(0.5)
        #self.tts_srv('/fd/return_table')
        self.bc.rotateAngle(-self.rotate)
        self.navi_srv('Tall table')
        rospy.sleep(0.5)
        self.bc.rotateAngle(self.rotate)
        rospy.sleep(0.5)
        self.tts_srv('/fd/finish_fd')

if __name__ == '__main__':
    rospy.init_node('fd_master')
    fd = FinalDemo()
