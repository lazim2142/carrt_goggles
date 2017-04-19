#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
import sys
import time
from std_msgs.msg import String
from face_recognition.msg import FRClientGoal

pub = rospy.Publisher('seen_names', String, queue_size = 10)

# really should be using a defaultDict, this is killing me inside
face_dict = {'cody':{'gone':True,'last_seen':0,'seen_count':0},
            'shamsi':{'gone':True,'last_seen':0,'seen_count':0},
            'alec':{'gone':True,'last_seen':0,'seen_count':0},
            'andres':{'gone':True,'last_seen':0,'seen_count':0}
            }

def callback(data):
    current_time = time.time()
    current_face = data.data

    if(current_face in face_dict):
        our_face = face_dict[current_face]
        our_face['last_seen'] = current_time
        our_face['seen_count'] +=1
        if our_face['gone'] and our_face['seen_count'] > 5:
            pub.publish(current_face + ' is in the room!')
            our_face['gone'] = False
            our_face['seen_count'] = 0

    # Detect if a friend has no longer been seen
    time_limit = 10
    for face in face_dict:
        cur_face = face_dict[face]
        if not cur_face['gone'] and current_time - cur_face['last_seen'] > time_limit:
            pub.publish(face + ' is no longer in the room!')
            cur_face['gone'] = True


def listener():
    facepub = rospy.Publisher('fr_order', FRClientGoal, queue_size=1)
    rospy.init_node('listener', anonymous=True)

    while(facepub.get_num_connections() <= 0): print "waiting for connection"

    msg, msg.order_id, msg.order_argument = FRClientGoal(), 1, "none"
    facepub.publish(msg)

    rospy.Subscriber('chatter', String, callback)
    rate = rospy.Rate(10)
    rospy.spin()

if __name__ == '__main__':
    listener()
