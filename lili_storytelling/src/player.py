

class Player:
    '''
    This class represents the player in the story.
    It contains a 'location' member that represents the current node
    in the traversal of the story graph.
    '''

    def __init__(self, root, list_of_StoryNodes, name = "John Doe"):

        #! Represents the player's location. Initialized at the root
        self.location = root 

        #! Dictionary containing the traversal status of each node (completed or not)
        self.completed = {}

        #! Name of the player. Can be used in dialog with a special escape sequence
        self.name = name

        # Puts all nodes in story in the completed dictionary as not completed
        for node in list_of_StoryNodes:
            self.completed[node.name] = False

    '''
    def setName(self):
        yes = 'no'
        while self.get_target(yes, ['yes'], yes_syns) != 'yes':
            speak("What is your name?")
            s = getInputString()
            speak("Is your name " + s + "?")
            yes = getInputString(yes_syns[0])
        return s
    '''
    
