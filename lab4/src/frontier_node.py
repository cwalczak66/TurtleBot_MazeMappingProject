#!/usr/bin/env python3
from __future__ import annotations
import rospy

class FrontierNode:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("frontier_node")