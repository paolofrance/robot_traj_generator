#!/usr/bin/env python

from __future__ import print_function

import rospy
from std_srvs import srv.SetBool





def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def bag_recorder():
    rospy.init_node('bag_recorder')
    s = rospy.Service('record_bag', AddTwoInts, handle_add_two_ints)
    print("Ready to record.")
    rospy.spin()

if __name__ == "__main__":
    bag_recorder()
