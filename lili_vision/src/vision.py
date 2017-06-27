#!/usr/bin/env python

import cv2
import sensor_msgs.msg
import cv_bridge
import rospy

class VisionNode:
    def __init__(self):
        self.sub = rospy.Subscriber('image', sensor_msgs.msg.Image, self.vision_handler)
        self.bridge = cv_bridge.CvBridge()

    def vision_handler(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

        cv2.imshow('image', img)
        cv2.waitKey(1)

def vision_server():
    rospy.init_node('vision')
    n = VisionNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()

if __name__ == '__main__':
    vision_server()
