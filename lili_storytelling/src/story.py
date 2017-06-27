from story_node import StoryNode
from player import Player

class Story(object):
    """
    This class represents a story which is built from StoryNodes
    and a Player. It takes a starting node root and a list of possible
    "ending" nodes. It then walks the tree with the player until the player
    reaches one of the end nodes.
    """

    def __init__(self, player, root, endings):
        #: The player instance that will walk through the story
        self.player = player
        #: The root node of the story, i.e the starting node
        self.root = root 
        #: A list of ending nodes for the story
        self.endings = endings

    def getNextNode(self, current, s):

        #check if user says exactly the node's name
        for child in current.children:
            if s.lower() == child.name.lower():
                return child

        #checks if user says node's name in a longer sentence (node's name can be multiple words)
        stems = []                          #get root of every word to compare                    
        for word in s.lower().split():
            stems.append(stemmer.stem(word))
        
        if "quit" in s:
            return None

        for c in current.children_stems:     #for every child of current
            count = 0
            for stem in c:                #for every word in current.child's name
                if stem in stems:         #if the user said that word
                    count += 1          #success, look for next word in name, if applicable
                                        #otherwise, check next child
            if count == len(c):         #if the user said every word in current.child's name
                return current.children[current.children_stems.index(c)]
        return current
        
    def prereqsValid(self, player, newCurrent):
        for prereq in newCurrent.prereqs:
            if not prereq in player.completed.keys() or player.completed[prereq] == False:
                speak("You do not have your " + prereq + " yet! Choose somewhere else to go.")
                return False
        return True


    def nextNode(self, player, lili):
        '''
        Takes player and liliAvatar as arguments. Does the speach associated
        with the current node, then takes the appropriate action associated
        with the connected edges. Returns thte next node that the player moves to. 
        '''

        current = player.location
		
        if current.image_path != None:
            print "Pretend I'm displaying image:", current.image_path
            #TODO Implement "Default" lili talking mode and alt image displaying mode


        if current.dialog is not None:
            lili.speak(current.dialog)
		
	# prompts user to choose next node out of options
        #speak("You are at the " + current.name + ". Where would you like to go now?")
        #self.printChildren(current)

	#if activity method did not return the next node, then get user's choice
        speechOptions = []
        for c in current.children:
            speechOptions.append(c.name)
            
        if s == None:
            s = getInputString(speechOptions)
	# check if a valid choice has been made or if getNextNode has returned the current working node
        newCurrent = self.getNextNode(current, s)
        while (not newCurrent == None) and (newCurrent == current or (not self.prereqsValid(player, newCurrent))):
            if newCurrent == current:
                speak("Sorry, that confused me. Please say one of the following:")
                for c in current.children:
                    speak(c.name)
            s = getInputString(speechOptions)
            newCurrent = self.getNextNode(current, s)
        #once a valid choice is made set current node then return it

        return newCurrent
    
            
    def walk(self, player, lili):
    '''
    'Main' function for the story
    Walks through the story, prompting the user when appropriate
    Exits when player's location is one of the 'ending' nodes specified
    in the constructor
    '''
        while player.location not in self.endings:
            player.location = self.nextNode(player, lili)

    def listChildren(self, current):
    '''
    Speaks the list of child edge names aloud
    '''
        for child in current.children:
            speak(child.name)

    def printStory(self):
    '''
    Prints the Story tree in order, listing each nodes children
    '''
        self.printStoryHelper(self.nodes[0])

    def printStoryHelper(self, current):
    '''
    Helper for printStory
    '''
        if current.children == []:
            pass

        print current.name, current.image_path, "Children: ",
        for c in current.children:
            print c.name,
        print '\n'

        for c in current.children:
            self.printStoryHelper(c)