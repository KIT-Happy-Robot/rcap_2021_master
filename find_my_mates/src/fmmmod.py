#/usr/bin/env python
# -*- coding: utf-8 -*-
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
        # self.feature_srv = rospy.ServiceProxy('get_feature_srv', StringToString)
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
            elif i == 3:
                self.name = "somebody" 
            else:
                tts_srv("Sorry. I'm going to ask you one more time.")
                i += 1
        return self.name

    def getAge(self):
        for i in range(3):
            age_res = self.feature_srv(req_data = "old")
            if age_res.result:
                self.age = age_res.res_data
                tts_srv(self.age)
            elif i == 3:
                self.age = "unknown"
            else:
                tts_srv("Sorry. I'm going to ask you one more time.")
                i += 1
        return self.age

    def getSex(self):
        tts_srv("Are you a female? Please answer with 'yes' or 'no'")
        result = self.yes_no_srv()
        if result:
            self.sex= "female"
        else:
            self.sex = "male"
        return self.sex


class LocInfo():
    def __init__(self):
        self.loc = "null"
        self.human_dict = rospy.get_param('/tmp_human_location')
        self.loc_dict = rospy.get_param('/location_dict')
        self.loc_name_list = list(self.loc_dict.keys())

    # 複数の座標のうちx, yに一番近い座標の名前を求める
    def nearPoint(self, target_name):
        # human_coord = self.human_dict[target_name]
        human_x = self.human_dict[target_name][0]
        human_y = self.human_dict[target_name][1]
        loc_x = self.loc_dict[self.loc_name[i]][0]
        loc_y = self.loc_dict[self.loc_name[i]][1]
        # stdval = math.sqrt((loc_x - human_x) ** 2 + (loc_y - human_y) ** 2)
        for i in range(len(self.loc_name_list)):
            # loc_coord = self.loc_dict[self.loc_name[i]]
            loc_x = self.loc_dict[self.loc_name[i]][0]
            loc_y = self.loc_dict[self.loc_name[i]][1]
            if i == 0:
                stdval = math.sqrt((loc_x - human_x) ** 2 + (loc_y - human_y) ** 2)
            distance = math.sqrt((loc_x - human_x) ** 2 + (loc_y - human_y) ** 2)
            if stdval > distance:
                    result = self.loc_name_list[i]
                    stdval = distance
        return result
