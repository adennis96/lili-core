import rospy

characterList = [None, "voice_cmu_us_bdl_arctic_clunits", "voice_cmu_us_clb_arctic_clunits", "voice_cmu_us_rms_arctic_clunits", "voice_cmu_us_slt_arctic_clunits"]

class Dialog:
    '''
    This class represents a single piece of dialog with an associated character.
    It contains a character ID (which determines the voice) and either a string
    or a filepath to a sound file. Defaults to the file if both are given.
    '''

    def __init__(self, characterID=0, text=None, filepath=None):

        try:
            #: A string representing the festival voice that will be used to read the text
            rospy.loginfo("Creating Dialog object")
            self.voiceName = characterList[characterID]
        except IndexError:
            self.voiceName = None
            rospy.logwarn("Invalid characterID passed to Dialog constuctor")

        # if text and filepath are both given, defaults to filepath, discards text
        if text is not None and filepath is not None:
            self.text = None
            self.filepath = filepath

        elif text is not None:
            self.text = text
            self.filepath = None

        # if both are None, then both will be set to None
        else:
            self.filepath = filepath
            self.text = None

