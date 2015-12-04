#!/usr/bin/env python

import rospy
from iw_tools import IWClient

rospy.init_node("test_iwc_goto")

iwc = IWClient()

print("Sending desire...")

iwc.add_desire(
    "GoTo", 
    params = {
        'frame_id': '/map',
        'x':        7.0,
        'y':        7.0,
        't':        0.0,
    },
    auto_rem = True
)

print("Spinning...")

rospy.spin()


