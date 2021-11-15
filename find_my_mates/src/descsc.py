#/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam

class NameInfoSC():
    def __init__(self):
        self.name = "null"
        self.name_srv = rospy.ServiceProxy('')

    def execute(self):
        slef.name = self.name_srv()
        return self.name


class LocInfoSC():
    def __init__(self, arg):
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



