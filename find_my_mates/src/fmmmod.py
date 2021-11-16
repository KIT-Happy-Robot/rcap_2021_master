#/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam
from happymimi_voice_msgs.srv import TTS, YesNo

# tts_srv
def tts_srv(phrase):
    srv = rospy.ServiceProxy('/tts', TTS)
    srv(phrase)


class NameInfoSC():
    def __init__(self):
        self.name = "null"
        self.name_srv = rospy.ServiceProxy('')

    def execute(self):
        slef.name = self.name_srv()
        return self.name


class LocInfo():
    def __init__(self, arg):
        self.loc = "null"
        self.human_dict = rospy.get_param('/tmp_human_location')
        self.loc_dict = rospy.get_param('/location_dict')
        self.loc_name_list = list(self.loc_dict.keys())

    # 複数の座標のうちx, yに一番近い座標の名前を求める
    @classmethod
    def nearPoint(self, target_name):
        # human_coord = self.human_dict[target_name]
        human_x = self.human_dict[target_name][0]
        human_y = self.human_dict[target_name][1]
        loc_x = self.loc_dict[self.loc_name[i]][0]
        loc_y = self.loc_dict[self.loc_name[i]][1]
        # stdval = math.sqrt(loc_x - human_x) ** 2 + (loc_y - human_y) ** 2)
        for i in range(len(self.loc_name_list)):
            # loc_coord = self.loc_dict[self.loc_name[i]]
            loc_x = self.loc_dict[self.loc_name[i]][0]
            loc_y = self.loc_dict[self.loc_name[i]][1]
            if i == 0:
                stdval = math.sqrt(loc_x - human_x) ** 2 + (loc_y - human_y) ** 2)
            distance = math.sqrt(loc_x - human_x) ** 2 + (loc_y - human_y) ** 2)
            if stdval > distance:
                    result = self.loc_name_list[i]
                    stdval = distance
        return result


class SexJudgment():
    def __init__(self):
        self.yes_no_srv = rospy.ServiceProxy('/yes_no', YesNo)
        self.sex_resut = "null"

    @classmethod
    def start(self):
        tts_srv("Excuse me")
        tts_srv("I have a question for you. Please answer with 'yes' or 'no'")
        tts_srv("Are you a female?")
        result = self.yes_no_srv()
        tts_srv("Thank you for your cooperation")
        if result:
            self.sex_resut = "female"
        else:
            self.sex_resut = "male"

