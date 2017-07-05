import time
import std_msgs.msg
import lili_audio.srv
import lili_audio.msg
from std_msgs.msg import String
import actionlib
import rospy
import dialog

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class LILIAvatar:
    '''
    This class represents the storytelling code's interface with the other nodes
    It contains methods that interface with the text to speech, voice recognition
    and graphics nodes.
    '''

    def __init__(self):
        self.speech_act = actionlib.SimpleActionClient('speech',
                                                       lili_audio.msg.TTSAction)
        self.speech_act.wait_for_server()
        rospy.wait_for_service('recognize_speech')
        self.listen_serv = rospy.ServiceProxy('recognize_speech',
                                              lili_audio.srv.RecognizeSpeech)
        self.display_pub = rospy.Publisher('display', std_msgs.msg.String,
                                           queue_size=10)

        self.sound_path = rospy.get_param('~sound_path')

        #: sound_play node for playing audio files
        self.soundhandle = SoundClient(blocking = True)

    def speak(self, dialog, block=True, display_lili=True):
        '''
        This method takes a dialog object or string text and sends it to the text to speech node.
        It has two optional bool args block and display_lili. Block determines if the
        method waits for the speech to finish before moving on. Display_lili determines
        if the method displays Lili talking on screen.
        '''
        if block and display_lili:
            self.display_pub.publish(std_msgs.msg.String('lili_talking.gif'))

        try:
            textToSay = dialog.text
        except AttributeError: #if dialog is a string rather than dialog object
            textToSay = dialog

        # If both text and filepath are None, nothing will be said.
        if textToSay is not None:

            goal = lili_audio.msg.TTSGoal(textToSay)

            self.speech_act.send_goal(goal)

            if block:
                self.speech_act.wait_for_result()

        elif dialog.filepath is not None:
            rospy.loginfo("Sound play request: " + dialog.filepath)
            self.soundhandle.playWave(self.sound_path + dialog.filepath)

        # Broken when block is False. Can't properly shut off Lili animation if
        # the method doesn't wait for speaking to finish
        if display_lili:
            self.display_pub.publish(std_msgs.msg.String('lili_idle.gif'))

    def listen(self):
        '''
        This method polls the speech recognition node.
        Returns a string representing the speech heard by the recognizer node
        '''
        resp = self.listen_serv(lili_audio.srv.RecognizeSpeechRequest())
        return resp.speech

    def display(self, image_path='lili_idle.gif'):
        '''
        This method displays an image specified by the path given by image_path
        '''
        self.display_pub.publish(std_msgs.msg.String(image_path))