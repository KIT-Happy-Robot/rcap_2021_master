#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam
from fmmmod import LocInfo

if __name__ == '__main__':
    rospy.init_node('fmmmod_test')
    li = LocInfo()
    result = li.nearPoint("human_1")
    print result
