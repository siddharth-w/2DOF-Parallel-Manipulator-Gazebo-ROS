import math
import numpy
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from model.msg import message1


def send(msg):
    if msg.data == True:
        msg1.linear.x = 0.1
        pub.publish(msg1)
        rospy.sleep(5)
        msg1.linear.x = 0.0
        msg.data = False
        pub.publish(msg1)
        pub1.publish(msg)


def move():

    rospy.init_node('move', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/job_done", Bool, send)

    rospy.spin()



if __name__ == '__main__':
    try:
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        pub1 = rospy.Publisher('job_done', Bool, queue_size=1)
        msg1 = Twist()

        move()
    except rospy.ROSInterruptException:
        pass