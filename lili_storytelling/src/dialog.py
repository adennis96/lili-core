
class Dialog:
    '''
    This class represents a single piece of dialog with an associated character.
    It contains a character ID (which determines the voice) and either a string
    or a filepath to a sound file. Defaults to the file if both are given.
    '''

    def __init__(self, characterID=0, text=None, filepath=None):

        self.characterID = characterID

        if text is not None and filepath is not None:
            self.text = None
            self.filepath = filepath

        elif text is not None:
            self.text = text
            self.filepath = None

        else:
            self.filepath = filepath
            self.text = None

