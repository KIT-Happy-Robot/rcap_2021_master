#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

class NameInfoSC():
    def __init__(self):
        self.name = "null"
        self.name_srv = rospy.ServiceProxy('')

    def execute(self):
        slef.name = self.name_srv()
        return self.name


class LocInfoSC():
    def __init__(self, arg):
        slef.target_num = arg
        self.loc = "null"

    # 複数の座標のうちx, yに一番近い座標を求める
    def nearPoint(x, y, points):
            result = {}
            if len(points) == 0:
                    return result
            result["x"] = points[0]["x"]
            result["y"] = points[0]["y"]
            stdval = math.sqrt((points[0]["x"] - x) ** 2 + (points[0]["y"] - y) ** 2)
            for point in points:
                    distance = math.sqrt((point["x"] - x) ** 2 + (point["y"] - y) ** 2)
                    if stdval > distance:
                            result["x"] = point["x"]
                            result["y"] = point["y"]
                            stdval = distance
            return result



