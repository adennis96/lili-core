#!/usr/bin/env python

import cv2
import sensor_msgs.msg
import cv_bridge
import rospy

class VisionNode:
    def __init__(self):
        self.sub = rospy.Subscriber('image', sensor_msgs.msg.Image, self.vision_handler)
        self.bridge = cv_bridge.CvBridge()
        self.cascade = cv2.CascadeClassifier(rospy.get_param('~cascade'))

    def vision_handler(self, data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        faces = self.cascade.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)

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
