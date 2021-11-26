#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import yaml
import roslib
import rospy
import rosparam
from fmmmod import LocInfo, FeatureFromRecog, FeatureFromVoice


class SaveInfo():
    def __init__(self):
        self.data_path = roslib.packages.get_pkg_dir("find_my_mates") + "/config/"

    def saveInfo(self, name, data):
        rospy.loginfo('Save feature')
        print self.data_path
        file_name = name + ".yaml"
        with open(os.path.join(self.data_path, file_name), "w") as yf:
            yaml.dump(data, yf, default_flow_style = False)


if __name__ == '__main__':
    rospy.init_node('fmmmod_test')
    # li = LocInfo()
    # ffr = FeatureFromRecog()
    # ffr = FeatureFromRecog()
    ffv = FeatureFromVoice()
    # result = ffv.getName()
    result = ffv.getAge()
    # result = ffv.getSex()
    # result = li.nearPoint("human_1")
    # result = ffr.getHeight()
    # data = ["a", "b", "c"]
    # si = SaveInfo()
    # si.saveInfo("human_1", data)
