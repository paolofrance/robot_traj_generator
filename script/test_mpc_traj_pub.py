#!/usr/bin/env python
import copy

import rospy
from geometry_msgs.msg import PoseArray, Pose
import numpy as np

def talker():
    human_pub = rospy.Publisher('/human_target', PoseArray, queue_size=10)
    robot_pub = rospy.Publisher('/target_cart_pose', PoseArray, queue_size=10)

    rospy.init_node('MPC_pub', anonymous=True)

    r=125
    rate = rospy.Rate(r)
    horizon=20

    init_pose = Pose()

    init_pose.position.x    = 0.11396073851419905
    init_pose.position.y    = -0.502474029164935
    init_pose.position.z    = -0.01910661257214613
    init_pose.orientation.x = -0.9951909923697849
    init_pose.orientation.y = -0.09786738903337923
    init_pose.orientation.z = -0.003607192592014885
    init_pose.orientation.w = 0.001962404503633281

    init_p=copy.deepcopy(init_pose)

    human_ref = PoseArray()
    robot_ref = PoseArray()

    for i in range(0, horizon):
        human_ref.poses.append(copy.deepcopy(init_pose))
        robot_ref.poses.append(copy.deepcopy(init_pose))
        
    # T=10 #sec
    # T_traj=np.array([0:dt:T])
    # x_ref_traj = init_p.position.x + 0.2*np.sin(T_traj)  # array di lunghezza len(T_traj)
    # y_ref_traj = init_p.position.y + 0.2*np.cos(T_traj)  # array di lunghezza len(T_traj)

    t=0

    while not rospy.is_shutdown():
        t+=1/r 

        for i in range(0, horizon):
            human_ref.poses[i].position.x = init_p.position.x + 0.2*np.sin(t+0.008*i)
            human_ref.poses[i].position.y = init_p.position.y + 0.2*np.cos(t+0.008*i)
            robot_ref.poses[i].position.x = init_p.position.x + 0.1*np.sin(t+0.008*i)
            robot_ref.poses[i].position.y = init_p.position.y + 0.1*np.cos(t+0.008*i)
            
            #robot_ref.poses[i].position.x = x_ref_traj[t+i]
            #robot_ref.poses[i].position.y = y_ref_traj[t+i]


        human_pub.publish(human_ref)
        robot_pub.publish(robot_ref)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
