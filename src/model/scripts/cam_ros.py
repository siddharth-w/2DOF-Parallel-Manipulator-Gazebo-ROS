#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import sys 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from model.msg import message1


pub = rospy.Publisher('pose', message1, queue_size=10)

msg = message1()

#land_point = CommandTOL()

# current_state = PoseStamped()


# def pose_cb(msg):
    # current_state = msg;


# state_sub  = rospy.Subscriber("mavros/state", State,state_cb)

# pos_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_cb)




bridge = CvBridge()
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
parameters = cv.aruco.DetectorParameters_create()

# parameters.cornerRefinementMethod=cv.aruco.CORNER_REFINE_SUBPIX

# detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

markerLength = 0.1

# focal_length=3.6
# width,length=(1080,720)    

cameraMatrix =np.array(  [[619.4827104748699,0.0,330.86654279772415],
                                        [0.0,618.5748144541332,276.5243455233969],
                                        [0.0,0.0,1.0]])
                                    
                                    
#cameraMatrix =np.array(  [[476.7030836014194,0.0,400.5],
                                    # [0.0,476.7030836014194,400.5],
                                    # [0.0,0.0,1.0]])                                         
distCoeffs = np.array( [[0.02800177528602659, 0.5453132261116592,0.013942705654592895,0.00796046432761273,-2.073190153728887]])

# distCoeffs = np.array( [[0, 0, 0, 0, 0]])











cap = cv.VideoCapture(0)
print(cap.isOpened())
if not cap.isOpened:
    print("error cam index error \n")
    sys.exit()








def p(img):
    ret, frame = cap.read()
    # frame = bridge.imgmsg_to_cv2(img, "passthrough")


    frame = cv.resize(frame , (640,480))
    (corners, ids, rejected) = cv.aruco.detectMarkers(frame, aruco_dict, parameters = parameters)

    if ids is not None:
        rvecs, tvecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs)
        # print("tvecs: ",tvecs)
        x,y,distance = tvecs[0][0]
        # print("x y :", y,x)

        # print("norm: ", np.linalg.norm(tvecs))


        # print(np.size(rvecs))
        if (np.size(rvecs) != 6):
            rotation_matrix,_= cv.Rodrigues(rvecs)
            
        

        sy = np.sqrt(rotation_matrix[0, 0] * rotation_matrix[0, 0] + rotation_matrix[1, 0] * rotation_matrix[1, 0])
        singular = sy < 1e-6

        if not singular:
        #     x = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
        #     y = np.arctan2(-rotation_matrix[2, 0], sy)
            yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            # x = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            # y = np.arctan2(-rotation_matrix[2, 0], sy)
            yaw = 0

        # print("yaw: ", yaw)

        for i in range(len(ids)):
            cv.aruco.drawDetectedMarkers(frame, corners)

            distance = np.linalg.norm(tvecs[i])


        print(tvecs)

        msg.theta1.data = -1000*y
        msg.theta2.data = 1000*x
        # print(distance)
        pub.publish(msg)




    cv.imshow("frame",frame)
    


    # print("Distance: ", distance,"roll : ",x ,"pitch :", y,"yaw", z)

    # x =0;
    if cv.waitKey(1) & 0xFF == ord("q"):
        cv.destroyAllWindows




    #pub = rospy.Publisher("camera_image_raw", Image, queue_size = 10)
    
    # while not rospy.is_shutdown():
        # ret,frame = cap.read()
        # if not ret:
        #     break



    # if len(corners) > 0:



    #     for (markerCorner, markerID) in zip(corners, id):
    #         corners = markerCorner.reshape((4, 2))
    #         (topLeft, topRight, bottomRight, bottomLeft) = corners


    #         topRight = (int(topRight[0]), int(topRight[1]))
    #         bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    #         bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    #         topLeft = (int(topLeft[0]), int(topLeft[1]))


    #         cv.line(frame, topLeft, topRight, (0, 255, 0), 2)
    #         cv.line(frame, topRight, bottomRight, (0, 255, 0), 2)
    #         cv.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
    #         cv.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
    # # compute and draw the center (x, y)-coordinates of the ArUco
    # # marker
    #         cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    #         cY = int((topLeft[1] + bottomRight[1]) / 2.0)
    #         cv.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
    # # draw the ArUco marker ID on the frame
    #         cv.putText(frame, str(markerID),
    #         (topLeft[0], topLeft[1] - 15), cv.FONT_HERSHEY_SIMPLEX,
    #         0.5, (0, 255, 0), 2)
    #         print("[INFO] ArUco marker ID: {}".format(markerID))
        
        
    #         if(markerID == 10):
    #             x = 10; 


    # if(x==10):

    #     land_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)

    #     land_client.call(altitude = 2, latitude = 0, longitude = 0, min_pitch = 0, yaw = 1.5707)
    #     # break;

if __name__ == '__main__':
    try:
        rospy.init_node("aruco_ros", anonymous = True)
        rate = rospy.Rate(10)

        sub_image = rospy.Subscriber("/camera/image_raw", Image , p)

        while not rospy.is_shutdown():
            rospy.spin()        
    except rospy.ROSInterruptException:
        pass