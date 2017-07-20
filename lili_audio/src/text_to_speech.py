#!/usr/bin/env python

from Queue import Queue
from gtts import gTTS
import lili_audio.msg
import actionlib
import rospy
import audio_common_msgs.msg


from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


class ROSSpeechStream:       # PRETTY SURE THIS IS ALL NOT NEEDED
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
        
        self.sound_path = rospy.get_param('~sound_path')

        #: sound_play node for playing audio files
        self.soundhandle = SoundClient(blocking = True)

        # ros parameters
        self.lang = rospy.get_param('~lang', 'en-us')
        self.slow = rospy.get_param('~slow', False)
        self.bitrate = rospy.get_param('~bitrate', 32000) # default for gTTS

    def speech_handler(self, goal):

        if not (goal.speech == ''):
            rospy.loginfo("TTS request: " + goal.speech)

            if goal.voice == '': # i.e. use google text to speech
                stream = ROSSpeechStream(self.bitrate)
                tts = gTTS(text=goal.speech, lang=self.lang, slow=self.slow) # gTTS saves to .mp3 files, sound_play plays .ogg and .wav
                tts.write_to_fp(stream)
                stream.publish(self.pub)

            else: 
                self.soundhandle.say(goal.speech, goal.voice)
        
        elif not (goal.filepath == ''):
            rospy.loginfo("Sound play request: " + goal.filepath)
            self.soundhandle.playWave(self.sound_path + goal.filepath)

        self.act.set_succeeded(lili_audio.msg.TTSResult())

def text_to_speech_server():
    rospy.init_node('text_to_speech')
    node = TTSServer()
    rospy.spin()

if __name__ == '__main__':
    text_to_speech_server()
