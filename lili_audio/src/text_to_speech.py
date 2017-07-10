#!/usr/bin/env python

from Queue import Queue
from gtts import gTTS
import audio_common_msgs.msg
import lili_audio.msg
import actionlib
import rospy

class ROSSpeechStream:
    def __init__(self, bitrate):
        self.bitrate = bitrate
        self.queue = Queue()

    def write(self, data):
        self.queue.put(data)

    def publish(self, pub):
        # queue up a few snippets so that the stream does not fall behind
        audio_snippet = self.queue.get()
        pub.publish(audio_common_msgs.msg.AudioData(audio_snippet))
        for i in range(4):
            pub.publish(audio_common_msgs.msg.AudioData(self.queue.get()))

        # publish the rest at a constant rate
        rate = rospy.Rate(self.bitrate/(8*len(audio_snippet)))
        while not self.queue.empty():
            audio_snippet = self.queue.get()
            pub.publish(audio_common_msgs.msg.AudioData(audio_snippet))
            rate.sleep()

class TTSServer:
    def __init__(self):

        # ros topics        
        self.pub = rospy.Publisher('audio', audio_common_msgs.msg.AudioData,
            queue_size=10)
        self.act = actionlib.SimpleActionServer('speech',
            lili_audio.msg.TTSAction, execute_cb=self.speech_handler,
            auto_start=False)
        self.act.start()

        # ros parameters
        self.lang = rospy.get_param('~lang', 'en-us')
        self.slow = rospy.get_param('~slow', False)
        self.bitrate = rospy.get_param('~bitrate', 32000) # default for gTTS

    def speech_handler(self, goal):

        rospy.loginfo("TTS request: " + goal.speech)

        # generate audio and stream out
        stream = ROSSpeechStream(self.bitrate)
        tts = gTTS(text=goal.speech, lang=self.lang, slow=self.slow)
        tts.write_to_fp(stream)
        stream.publish(self.pub)

        self.act.set_succeeded(lili_audio.msg.TTSResult())

def text_to_speech_server():
    rospy.init_node('text_to_speech')
    node = TTSServer()
    rospy.spin()

if __name__ == '__main__':
    text_to_speech_server()
