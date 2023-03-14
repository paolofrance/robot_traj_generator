#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

def sin_pub():
    rospy.init_node('sinpub', anonymous=True)
    pub = rospy.Publisher('/target_cart_pose', PoseStamped, queue_size=10)

    alp_pub = rospy.Publisher('/alpha', Float32, queue_size=10)

    hz=125

    rate = rospy.Rate(hz) # 10hz

    x_init = 0.11377141733208844

    pose_msg = PoseStamped()
    pose_msg.pose.position.x = 0.11377141733208844
    pose_msg.pose.position.y = -0.5024423604441172
    pose_msg.pose.position.z = 0.02
    pose_msg.pose.orientation.x = -0.9951897224119687
    pose_msg.pose.orientation.y = -0.09787904169232702
    pose_msg.pose.orientation.z = -0.003622292316069252
    pose_msg.pose.orientation.w = 0.0019971483062902044

    t=0
    t_ramp=0

    dt1=5
    dt2=dt1+1
    dt3=dt2+5
    dt4=dt3+1

    al=1

    while not rospy.is_shutdown():
        t += 1 / hz
        t_ramp += 1 / hz

        pose_msg.pose.position.x = x_init + 0.3*np.sin(3*t)
        pub.publish(pose_msg)

        # al = 0.5 + 0.5*(0.98 * np.sin(0.1*t))

        if t_ramp < dt1:
            al=1
        elif dt1<t_ramp<dt2:
            al = 1-(dt2-dt1)*(t_ramp-dt1)*0.98
        elif dt2<t_ramp<dt3:
            al = 0.01
        elif dt3<t_ramp<dt4:
            al = 0.01+(dt4-dt3)*(t_ramp-dt3)*0.98
        else:
            t_ramp=0


        alp_pub.publish(al)

        rate.sleep()

if __name__ == '__main__':
    try:
        sin_pub()
    except rospy.ROSInterruptException:
        pass