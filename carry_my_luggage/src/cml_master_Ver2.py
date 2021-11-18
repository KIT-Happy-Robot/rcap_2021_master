#!/usr/bin/env python
# -*- coding: utf-8 -*-
#----------------------------------------------------
# Title: RCAP2021 Carry My Luggage
# Author: Yusuke Kanazawa
# Date: 2021/11/
# Memo:
#----------------------------------------------------
import rospy
import smach
import smach_ros
from std_msgs.msg import String, Float64
from happymimi_robot.srv import StrTrg
from happymimi_voice_msgs.srv import YesNo
from happymimi_navigation.srv import NaviLocation
from happymimi_msgs.srv import StrTrg

# tts_srv
tts_srv = rospy.ServiceProxy('/tts', StrTrg)


class GraspOrPass(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['GOP_finish'],
                                   input_keys = ['GOP_count_in'])
        # Publisher
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        # ServiceProxy
        self.arm_srv = rospy.ServiceProxy('/servo/arm', StrTrg)

    def execute(self, userdata):
        rospy.loginfo('Executing state: GRASP_OR_PASS')
        if userdata.GOP_count_in == 0:
            self.head_pub.publish(10)
            tts_srv('Please give it to me')
            result = self.arm_srv('receive').result
            if result:
                tts_srv('Thank You')
                self.arm_srv('carry')
                self.head_pub.publish(0)
                return 'GRASP_finish'
            else:
                tts_srv("Sorry, I couldn't receive it")
                tts_srv("Please give it to me again")
                return 'GRASP_failure'
        else:
            self.head_pub.publish(10)
            tts_srv('Here you are')
            self.arm_srv('give')
            self.head_pub.publish(0)
            return 'PASS_finish'


class Chaser(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['chaser_finish'],
                                   input_keys = ['PASS_count_in'],
                                   output_keys = ['PASS_count_out'])
        # Publisher
        self.chaser_pub = rospy.Publisher('/follow_human', String, queue_size = 1)
        # Subscriber
        rospy.Subscriber('/find_str', String, self.findCB)
        # ServiceProxy
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)
        # Value
        self.find_msg = False

    def findCB(self, receive_msg):
        self.find_msg = receive_msg.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: CHASER')
        pass_count = userdata.PASS_count_in
        tts_srv("I'll follow you")
        tts_srv("Please come in front of me")
        self.chaser_pub.publish('start')
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.find_msg == "lost":
                tts_srv("I lost sight of you")
                tts_srv("Is this the location of the car?")
                answer = self.yesno_srv().result
                if answer:
                    self.chaser_pub.publish('stop')
                    userdata.PASS_count_out += 1
                    return 'chaesr_finish'
                else:
                    tts_srv("Ok, continue to follow")
                    tts_srv("Please come in front of me")
                    self.find_msg = "NULL"
            else:
                pass


class Return(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['return_finish'])
        # Service
        self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)

    def execute(self, userdata):
        rospy.loginfo('Executing state: RETURN')
        rospy.sleep(0.5)
        self.navi_srv('operator')
        return 'return_finish'


if __name__=='__main__':
    rospy.init_node('cml_master')
    rospy.loginfo("Start Carry My Luggage")
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    sm_top.userdata.GOP_count = 0
    with sm_top:
        smach.StateMachine.add(
                'GRASP_OR_PASS',
                GraspOrPass(),
                transitions = {'GRASP_finish':'CHASER',
                               'PASS_finish':'RETURN',
                               'GRASP_failure':'GRASP_OR_PASS'},
                remapping = {'GOP_count_in':'GOP_count'})

        smach.StateMachine.add(
                'CHASER',
                Chaser(),
                transitions = {'chaser_finish':'GRASP_OR_PASS'},
                remapping = {'PASS_count_in':'GOP_count',
                             'PASS_count_out':'GOP_count'})

        smach.StateMachine.add(
                'RETURN',
                Return(),
                transitions = {'return_finish':'finish_sm_top'})

        outcome = sm_top.execute()
