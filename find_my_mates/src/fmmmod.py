#!/usr/bin/env python
# -*- coding: utf-8 -*-
from scipy.spatial import distance
import tf
import rospy
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
        self.name = "null"
        for i in range(3):
            name_res = self.feature_srv(req_data = "name")
            if name_res.result:
                self.name = name_res.res_data
                tts_srv("Hi " + self.name)
                break
            elif i == 3:
                self.name = "somebody"
                break
            else:
                tts_srv("Sorry. I'm going to ask you one more time.")
        return self.name

    def getAge(self):
        self.age = "null"
        for i in range(3):
            age_res = self.feature_srv(req_data = "old")
            if i == 3:
                self.age + "unknown"
                break
            elif age_res.result:
                self.age = age_res.res_data
                tts_srv("Your age is" + self.age)
                tts_srv("Is this OK? Please answer yes or no")
                if self.yesNo():
                    break
                else:
                    tts_srv("Sorry. I'm going to ask you one more time.")
            else:
                tts_srv("Sorry. I'm going to ask you one more time.")
        return self.age

    def getSex(self):
        self.sex = "null"
        tts_srv("Are you a female? Please answer with yes or no")
        result = self.yes_no_srv().result
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

    # 複数の座標のうちx, yに一番近い座標の名前を求める
    def nearPoint(self, target_name):
        self.loc_name = "null"
        self.human_dict = rospy.get_param('/tmp_human_location')
        h_rpy = self.human_dict[target_name]
        h_xy = (h_rpy[0], h_rpy[1])
        for i in range(len(self.loc_name_list)):
            self.loc_name = self.loc_name_list[i]
            loc_rpy = self.loc_dict[self.loc_name]
            l_xy = (loc_rpy[0], loc_rpy[1])
            if i == 0:
                stdval = distance.euclidean(h_xy, l_xy)
            dist = distance.euclidean(h_xy, l_xy)
            print self.loc_name
            print dist
            if stdval > dist:
                stdval = dist
                loc_result = self.loc_name
        print loc_result
        return loc_result
