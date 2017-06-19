#!/usr/bin/env python

from gtts import gTTS
import std_msgs.msg
import audio_common_msgs.msg
import rospy

class ROSSpeechStream:
    def __init__(self, pub):
        self.pub = pub

    def write(self, data):
        self.pub.publish(audio_common_msgs.msg.AudioData(data))

class TTSServer:
    def __init__(self):
        self.pub = rospy.Publisher('audio', audio_common_msgs.msg.AudioData, queue_size=100)
        self.sub = rospy.Subscriber('speech', std_msgs.msg.String, self.speech_handler)
        self.lang = 'en'
        self.stream = ROSSpeechStream(self.pub)

    def speech_handler(self, data):
        rospy.loginfo("TTS request: " + data.data)
        tts = gTTS(text=data.data, lang=self.lang)
        tts.write_to_fp(self.stream)

def text_to_speech_server():
    rospy.init_node('speech recognition')
    node = TTSServer()
    rospy.spin()

if __name__ == '__main__':
    text_to_speech_server()
