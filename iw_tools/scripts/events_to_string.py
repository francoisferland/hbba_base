#!/usr/bin/env python

# Serializes HBBA events into a single string

import rospy
from std_msgs.msg import String
from hbba_msgs.msg import Event

def sub_events(msg):
    global pub_evt_string
    evt_string = "hbba_event_"          \
               + str(msg.type)          \
               + "_" + msg.desire_type  \
               + "_" + msg.desire

    pub_evt_string.publish(evt_string)

if __name__ == '__main__':
    global pub_evt_string

    rospy.init_node("event_to_string")

    pub_evt_string = rospy.Publisher("/hbba/events_string", String, queue_size=10)
    sub_events     = rospy.Subscriber("/hbba/events", Event, sub_events)

    rospy.spin()

