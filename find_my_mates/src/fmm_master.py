#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rosparam
import smach
import smach_ros
from fmmmod import FeatureFromVoice, LocInfo
from std_msgs.msg import Float64
from happymimi_msgs.srv import SimpleTrg, StrTrg
from happymimi_navigation.srv import NaviLocation, NaviCoord

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
        # Param
        # self.coord_dict = rospy.get_param('/tmp_human_location')

    def execute(self, userdata):
        rospy.loginfo("Executing state: APPROACH_GUEST")
        return 'approach_finish'
        guest_num = userdata.g_count_in
        guest_name = "human_" + str(guest_num)
        # self.navi_srv('living room')
        self.head_pub.publish(10)
        if guest_num == 0:
            self.gen_coord_srv()
            coord_dict = rospy.get_param('/tmp_human_location')
        else:
            pass
        tts_srv("Move to guest")
        print guest_name
        result = self.ap_srv(data = guest_name)
        if result:
            self.head_pub.publish(0)
            return 'approach_finish'
        else:
            self.head_pub.publish(0)
            return 'approach_finish'


class FindFeature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['find_finish'],
                             input_keys = ['g_count_in'],
                             output_keys = ['future_out'])
        self.ffv = FeatureFromVoice()
        self.guest_name  = "null"
        self.guest_loc   = "null"
        self.gn_sentence = "null"
        self.f1_sentence = "null"
        self.f2_sentence = "null"
        self.sentence_list = []

    def execute(self, userdata):
        rospy.loginfo("Executing state: FIND_FUATURE")

        self.guest_name = self.ffv.getName()
        # self.guest_loc = LocINfo.nearPoint(num = userdata.g_count_in)
        # self.gn_sentence = self.guest_name + " is near " + self.guest_loc
        print self.guest_name
        self.gn_sentence = (self.guest_name + " is near table")
        # moduleを作る（サービスのクライアントまとめたやつ）
        if userdata.g_count_in == 0:
            rospy.loginfo("Hello")
            self.f1_sentence = self.ffv.getSex()
            self.f2_sentence = self.ffv.getAge()
        elif userdata.g_count_in == 1:
            # self.f1_sentence = HeightInfoSC()
            # self.f2_sentence = ClothesInfoSC()
            pass
        else:
            return 'find_finish'

        userdata.future_out = [self.gn_sentence, self.f1_sentence, self.f2_sentence]
        return 'find_finish'


class TellFeature(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['tell_finish', 'all_finish'],
                             input_keys = ['g_count_in', 'future_in'],
                             output_keys = ['g_count_out'])
        self.sentence_list = []
        # Service
        self.navi_srv = rospy.ServiceProxy('navi_location_server', NaviLocation)

    def execute(self, userdata):
        rospy.loginfo("Executing state: TELL_FUATURE")
        guest_num = userdata.g_count_in
        self.sentence_list = userdata.future_in
        print self.sentence_list
        if guest_num > 2:
            tts_srv("Finish Find My Mates. Thank you very much")
            return 'all_finish'
        else:
            self.navi_srv('operator')
            pass
        for i in range(len(self.sentence_list)):
            phrase = self.sentence_list[i]
            print phrase
            tts_srv(phrase)
            rospy.sleep(0.1)
            i += 1
        userdata.g_count_out = guest_num + 1
        return 'tell_finish'


if __name__ == '__main__':
    rospy.init_node('fmm_master')
    rospy.loginfo("Start Find My Mates")
    tts_srv("Start Find My Mates")
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    sm_top.userdata.guest_count = 0
    with sm_top:
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
                transitions = {'tell_finish':'APPROACH_GUEST',
                               'all_finish':'finish_sm_top'},
                remapping = {'future_in':'guest_future',
                             'g_count_in':'guest_count',
                             'g_count_out':'guest_count'})

    outcome = sm_top.execute()
