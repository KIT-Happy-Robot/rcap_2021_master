#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosparam
import descsc


if __name__ == '__main__':
   lisc = LocINfoSC() 
   result = lisc.nearPoint('human_0')
   print result
