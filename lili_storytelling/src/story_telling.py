#!/usr/bin/env python

import time
import lili_audio.srv
import std_msgs.msg
import rospy

class LILIAvatar:
    def __init__(self):
        self.speech_pub = rospy.Publisher('speech', std_msgs.msg.String, queue_size=10)
        rospy.wait_for_service('recognize_speech')
        self.listen_serv = rospy.ServiceProxy('recognize_speech', lili_audio.srv.RecognizeSpeech)

    def speak(self, text):
        speech = std_msgs.msg.String(text)
        self.speech_pub.publish(speech)

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
