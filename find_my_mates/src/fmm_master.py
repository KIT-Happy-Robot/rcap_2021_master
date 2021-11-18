#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import rosparam
import roslib
import smach
import smach_ros
from fmmmod import FeatureFromVoice, LocInfo
from std_msgs.msg import Float64
from happymimi_msgs.srv import SimpleTrg, StrTrg
from happymimi_navigation.srv import NaviLocation, NaviCoord

file_path = roslib.packages.get_pkg_dir('happymimi_teleop') + '/src/'
sys.path.insert(0, file_path)
from base_control import BaseControl

# speak
tts_srv = rospy.ServiceProxy('/tts', StrTrg)


class ApproachGuest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['approach_finish'],
                             input_keys = ['g_count_in'])
        # Service
        self.gen_coord_srv = rospy.ServiceProxy('/human_coord_generator', SimpleTrg)
        self.ap_srv = rospy.ServiceProxy('/approach_person_server', StrTrg)
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        # Topic
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.bc = BaseControl()

    def execute(self, userdata):
        rospy.loginfo("Executing state: APPROACH_GUEST")
        guest_num = userdata.g_count_in
        guest_name = "human_" + str(guest_num)
        tts_srv("Move to guest")
        if guest_num == 0:
            self.navi_srv('living room')
            self.head_pub.publish(10)
            self.bc.rotateAngle(-20)
            self.gen_coord_srv()
            # for i in range(2):
                # rospy.sleep(2.0)
                # self.gen_coord_srv()
                # self.bc.rotateAngle(-90)
        else:
            pass
        result = self.ap_srv(data = guest_name)
        self.head_pub.publish(0)
        if result:
            return 'approach_finish'
        else:
            return 'approach_finish'


class FindFeature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['find_finish'],
                             input_keys = ['g_count_in'],
                             output_keys = ['future_out'])
        # Topic
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        self.li = LocInfo()
        self.ffv = FeatureFromVoice()
        self.guest_name  = "null"
        self.guest_loc   = "null"
        self.gn_sentence = "null"
        self.f1_sentence = "null"
        self.f2_sentence = "null"
        self.sentence_list = []

    def execute(self, userdata):
        rospy.loginfo("Executing state: FIND_FUATURE")
        self.head_pub.publish(-20)
        tts_srv("Excuse me. I have a question for you")
        self.guest_name = self.ffv.getName()
        print self.guest_name
        self.guest_loc = self.li.nearPoint("human_" + str(userdata.g_count_in))
        self.gn_sentence = self.guest_name + " is near " + self.guest_loc
        # self.gn_sentence = (self.guest_name + " is near table")
        if userdata.g_count_in == 0:
            self.f1_sentence = "Gender is " + self.ffv.getSex()
            self.f2_sentence = "Age is " + self.ffv.getAge()
        elif userdata.g_count_in == 1:
            # self.f1_sentence = HeightInfoSC()
            # self.f2_sentence = ClothesInfoSC()
            self.f1_sentence = "Gender is " + self.ffv.getSex()
            self.f2_sentence = "Age is " + self.ffv.getAge()
            pass
        else:
            return 'find_finish'
        tts_srv("Thank you for your cooperation")
        userdata.future_out = [self.gn_sentence, self.f1_sentence, self.f2_sentence]
        return 'find_finish'


class TellFeature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['tell_finish'],
                             input_keys = ['g_count_in', 'future_in'],
                             output_keys = ['g_count_out'])
        # Service
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)
        # Topic
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        # Value
        self.sentence_list = []

    def execute(self, userdata):
        rospy.loginfo("Executing state: TELL_FUATURE")
        guest_num = userdata.g_count_in
        self.sentence_list = userdata.future_in
        tts_srv("Move to operator")
        navi_result = self.navi_srv('operator').result
        self.head_pub.publish(-20)
        if navi_result:
            tts_srv("I'll give you the guest information.")
        else:
            tts_srv("I'm sorry. I couldn't navigate to the operator's location. I will provide the features from here.")
        print self.sentence_list
        for i in range(len(self.sentence_list)):
            tts_srv(self.sentence_list[i])
            i += 1
        userdata.g_count_out = guest_num + 1
        return 'tell_finish'


class Operation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['start_test', 'all_finish'],
                             input_keys = ['g_count_in'])

    def execute(self, userdata):
        rospy.loginfo("Executing state: OPERATION")
        guest_count = userdata.g_count_in
        if guest_count == 0:
            tts_srv("Start Find My Mates")
            return 'start_test'
        elif guest_count > 1:
            tts_srv("Finish Find My Mates. Thank you very much")
            return 'all_finish'
        else:
            return 'start_test'


if __name__ == '__main__':
    rospy.init_node('fmm_master')
    rospy.loginfo("Start Find My Mates")
    # tts_srv("Start Find My Mates")
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    sm_top.userdata.guest_count = 0
    with sm_top:
        smach.StateMachine.add(
                'OPERATION',
                Operation(),
                transitions = {'start_test':'APPROACH_GUEST',
                               'all_finish':'finish_sm_top'},
                remapping = {'g_count_in':'guest_count'})

        smach.StateMachine.add(
                'APPROACH_GUEST',
                ApproachGuest(),
                transitions = {'approach_finish':'FIND_FEATURE'},
                remapping = {'g_count_in':'guest_count'})

        smach.StateMachine.add(
                'FIND_FEATURE',
                FindFeature(),
                transitions = {'find_finish':'TELL_FEATURE'},
                remapping = {'future_out':'guest_future',
                             'g_count_in':'guest_count'})

        smach.StateMachine.add(
                'TELL_FEATURE',
                TellFeature(),
                transitions = {'tell_finish':'OPERATION'},
                remapping = {'future_in':'guest_future',
                             'g_count_in':'guest_count',
                             'g_count_out':'guest_count'})

    outcome = sm_top.execute()
