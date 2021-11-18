#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import tf
import rosparam
from happymimi_msgs.srv import SimpleTrg, StrTrg, StrToStr
from happymimi_voice_msgs.srv import TTS, YesNo

# tts_srv
tts_srv = rospy.ServiceProxy('/tts', StrTrg)

class FeatureFromVoice():
    def __init__(self):
        # Service
        self.feature_srv = rospy.ServiceProxy('get_feature_srv', StrToStr)
        self.yes_no_srv = rospy.ServiceProxy('/yes_no', YesNo)
        # Value
        self.name = "null"
        self.age  = "null"
        self.sex  = "null"

    def yesNo(self):
        result = self.yes_no_srv().result
        return result

    def getName(self):
        for i in range(3):
            name_res = self.feature_srv(req_data = "name")
            if name_res.result:
                self.name = name_res.res_data
                tts_srv("Hi " + self.name)
                break
            elif i == 3:
                self.name = "somebody"
            else:
                tts_srv("Sorry. I'm going to ask you one more time.")
                # i += 1
        return self.name

    def getAge(self):
        for i in range(3):
            age_res = self.feature_srv(req_data = "old")
            if age_res.result:
                self.age = age_res.res_data
                tts_srv("Your age is" + self.age)
                tts_srv("Is this OK? Please answer 'yes or 'no")
                if self.yesNo():
                    break
                else:
                    tts_srv("Sorry. I'm going to ask you one more time.")
                    # i += 1
            elif i == 3:
                self.age = "unknown"
            else:
                tts_srv("Sorry. I'm going to ask you one more time.")
                # i += 1
        return self.age

    def getSex(self):
        tts_srv("Are you a female? Please answer with 'yes' or 'no'")
        result = self.yes_no_srv()
        if result:
            self.sex= "female"
        else:
            self.sex = "male"
        tts_srv("You are " + self.sex)
        return self.sex


class LocInfo():
    def __init__(self):
        self.loc_dict   = rospy.get_param('/location')
        self.human_dict = {}
        self.loc_name_list = list(self.loc_dict.keys())
        self.loc_name      = "null"

    def quat_to_rpy(self, q):
        rpy = tf.transformations.euler_from_quaternion((q[0], q[1], q[2], q[3]))
        rpy = [round(rpy[0], 3), round(rpy[1], 3), round(rpy[2], 3)]
        return rpy

    # 複数の座標のうちx, yに一番近い座標の名前を求める
    def nearPoint(self, target_name):
        self.loc_name = "null"
        self.human_dict = rospy.get_param('/tmp_human_location')
        h_rpy = self.quat_to_rpy(self.human_dict[target_name])
        human_x = h_rpy[0]
        human_y = h_rpy[1]
        for i in range(len(self.loc_name_list)):
            self.loc_name = self.loc_name_list[i]
            loc_rpy = self.quat_to_rpy(self.loc_dict[self.loc_name])
            loc_x = loc_rpy[0]
            loc_y = loc_rpy[1]
            if i == 0:
                stdval = math.sqrt((loc_x - human_x) ** 2 + (loc_y - human_y) ** 2)
            distance = math.sqrt((loc_x - human_x) ** 2 + (loc_y - human_y) ** 2)
            elif stdval > distance:
                    stdval = distance
                    loc_result = self.loc_name
        print loc_result
        return loc_result
