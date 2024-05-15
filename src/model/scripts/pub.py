import math
import numpy
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from model.msg import message1


def send():
    pub = rospy.Publisher('pose', message1, queue_size=10)
    rospy.init_node('sender', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = message1()
        # print(msg)
        msg.theta1.data = -100
        msg.theta2.data = 100
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass