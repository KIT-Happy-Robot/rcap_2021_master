#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam
from fmmmod import LocInfo, FeatureFromRecog

if __name__ == '__main__':
    rospy.init_node('fmmmod_test')
    # li = LocInfo()
    ffr = FeatureFromRecog()
    # result = li.nearPoint("human_1")
    result = ffr.getHeight()
    print result
