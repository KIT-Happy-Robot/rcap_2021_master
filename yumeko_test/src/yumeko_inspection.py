#!/usr/bin/env python
# -*- coding: utf-8 -*-
#----------------------------------------------------------
# Title: RCAP2021見極め用マスター
# Author: Issei Iida
# Date: 2021/10/22
# Memo: 競技内容はGPSR(RCJ2019仕様)
#----------------------------------------------------------
import sys
import rospy
import roslib
import actionlib
import smach
import smach_ros

from std_msgs.msg import String, Float64
from happymimi_navigation.srv import NaviLocation
from enter_room.srv import EnterRoom
from happymimi_voice_msgs.srv import TTS, YesNo, ActionPlan
from actplan_executor.msg import APExecutorAction, APExecutorGoal

# tts_srv
tts_srv = rospy.ServiceProxy('/tts', TTS)


class Enter(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['enter_finish'])
        # Service
        self.enter_srv = rospy.ServiceProxy('enter_room_server', EnterRoom)

    def execute(self, userdata):
        rospy.loginfo("Executing state: ENTER")
        tts_srv("Start GPSR")
        self.enter_srv(distance=0.8, velocity=0.2)
        return 'enter_finish'


class DecideMove(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['decide_finish', 'all_cmd_finish'],
                             input_keys = ['cmd_count_in'])
        # Service
        self.navi_srv = rospy.ServiceProxy('/navi_location_server', NaviLocation)
        # Subscriber
        self.posi_sub = rospy.Subscriber('/current_location', String, self.crPosiCB)
        # Value
        self.current_loc = 'none'
        self.cmd_limit = 3

    def crPosiCB(self, data):
        self.current_loc = data.data

    def execute(self, userdata):
        rospy.loginfo('Executing state: DECIDE_MOVE')
        if userdata.cmd_count_in == self.cmd_limit + 1:
            tts_srv("Finish all command, Move to entrance")
            self.navi_srv('entrance')
            tts_srv("Finish GPSR")
            return 'all_cmd_finish'
        elif self.current_loc != 'operator':
            self.navi_srv('operator')
            return 'decide_finish'
        else:
            return 'decide_finish'


class ListenCommand(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['listen_success',
                                         'listen_failure',
                                         'next_cmd'],
                             input_keys = ['cmd_count_in'],
                             output_keys = ['cmd_out_action',
                                            'cmd_out_data',
                                            'cmd_count_out'])
        # Publisher
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        # ServiceProxy
        # self.listen_srv = rospy.ServiceProxy('/gpsr/actionplan', ActionPlan)
        self.listen_srv = rospy.ServiceProxy('/planning_srv', ActionPlan)
        self.yesno_srv = rospy.ServiceProxy('/yes_no', YesNo)
        # Value
        self.listen_count = 1
        # kimeuti actplan
        self.apdemo_action = [['go','grasp','go','give'],
                        ['go','grasp','go','give'],
                        ['go','grasp','go','give']]
        self.apdemo_data = [['cupboard','black coffee','operator','black coffee'],
                       ['cupboard','red cup','operator','red cup'],
                       ['cupboard','green snack','operator','green snack']]

    def execute(self, userdata):
        rospy.loginfo("Executing state: LISTEN_COMMAND")
        cmd_count = userdata.cmd_count_in
        self.head_pub.publish(0)
        if self.listen_count <= 3:
            tts_srv("CommandNumber is " + str(cmd_count))
            tts_srv("ListenCount is " + str(self.listen_count))
            tts_srv("Please instruct me")
            actplan_res = self.listen_srv()
            # actplan_res = True
            if actplan_res.result:
            # if actplan_res:
                tts_srv("Is this correct?")
                answer = self.yesno_srv().result
                if answer:
                    userdata.cmd_out_action = actplan_res.action
                    userdata.cmd_out_data = actplan_res.data
                    # userdata.cmd_out_action = self.apdemo_action[cmd_count - 1]
                    # userdata.cmd_out_data = self.apdemo_data[cmd_count -1]
                    self.listen_count = 1
                    cmd_count += 1
                    userdata.cmd_count_out = cmd_count
                    return 'listen_success'
                else:
                    tts_srv("Sorry")
                    self.listen_count += 1
                    return 'listen_failure'
            else:
                self.listen_count += 1
                tts_srv("I could't listen")
                return 'listen_failure'
        else:
            tts_srv("I couldn't understand the instruction")
            self.listen_count = 1
            cmd_count +=1
            userdata.cmd_count_out = cmd_count
            return 'next_cmd'


class ExeAction(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes = ['action_success',
                                         'action_failure'],
                             input_keys = ['cmd_in_action',
                                           'cmd_in_data'])

    def execute(self, userdata):
        rospy.loginfo("Executing state: EXE_ACTION")
        ac = actionlib.SimpleActionClient('actplan_executor', APExecutorAction)
        ac.wait_for_server()
        # Goalの生成
        goal = APExecutorGoal()
        goal.action = userdata.cmd_in_action
        goal.data = userdata.cmd_in_data
        print goal
        ac.send_goal(goal)
        ac.wait_for_result()
        result = ac.get_result().data
        if result == 'success':
            rospy.loginfo("Success ExeActionPlan")
            return 'action_success'
        else:
            rospy.loginfo("Failed ExeActionPlan")
            return 'action_failure'


if __name__ == '__main__':
    rospy.init_node('yumeko_inspection')
    sm_top = smach.StateMachine(outcomes = ['finish_sm_top'])
    sm_top.userdata.cmd_count = 1
    with sm_top:
        smach.StateMachine.add(
                'ENTER',
                Enter(),
                transitions = {'enter_finish':'DECIDE_MOVE'})

        smach.StateMachine.add(
                'DECIDE_MOVE',
                DecideMove(),
                transitions = {'decide_finish':'LISTEN_COMMAND',
                               'all_cmd_finish':'finish_sm_top'},
                remapping = {'cmd_count_in':'cmd_count'})

        smach.StateMachine.add(
                'LISTEN_COMMAND',
                ListenCommand(),
                transitions = {'listen_success':'EXE_ACTION',
                               'listen_failure':'LISTEN_COMMAND',
                               'next_cmd':'DECIDE_MOVE'},
                remapping = {'cmd_out_action':'ap_action',
                             'cmd_out_data':'ap_data',
                             'cmd_count_in':'cmd_count',
                             'cmd_count_out':'cmd_count'})

        smach.StateMachine.add(
                'EXE_ACTION',
                ExeAction(),
                transitions = {'action_success':'DECIDE_MOVE',
                               'action_failure':'DECIDE_MOVE'},
                remapping = {'cmd_in_action':'ap_action',
                             'cmd_in_data':'ap_data'})

    outcome = sm_top.execute()
