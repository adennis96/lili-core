#!/usr/bin/env python

import time
import lili_audio.srv
import lili_audio.msg
from std_msgs.msg import String
import actionlib
import rospy

class LILIAvatar:
    def __init__(self):
        self.speech_act = actionlib.SimpleActionClient('speech', lili_audio.msg.TTSAction)
        rospy.wait_for_service('recognize_speech')
        self.listen_serv = rospy.ServiceProxy('recognize_speech', lili_audio.srv.RecognizeSpeech)
        self.display_pub = rospy.Publisher('display', String, queue_size = 10)

    def speak(self, text, block=True):
        goal = lili_audio.msg.TTSGoal(text)
        self.speech_act.send_goal(goal)
        if block:
            self.speech_act.wait_for_result()

    def listen(self):
        resp = self.listen_serv(lili_audio.srv.RecognizeSpeechRequest())
        return resp.speech


def test():
    rospy.init_node('story_telling')
    avatar = LILIAvatar()
    time.sleep(1)
    avatar.speak('Hi! My name is Lili. What is yours?')
    name = avatar.listen()
    avatar.speak('Hi ' + name + ', nice to meet you.')

if __name__ == '__main__':
    test()
