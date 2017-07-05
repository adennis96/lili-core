from story_node import StoryNode
from player import Player
from lili_avatar import LILIAvatar
from nltk.stem.snowball import SnowballStemmer
import time

stemmer = SnowballStemmer('english')

class Story(object):
    """
    This class represents a story which is built from StoryNodes
    and a Player. It takes a starting node root and a list of possible
    "ending" nodes. It then walks the tree with the player until the player
    reaches one of the end nodes.
    """

    def __init__(self, player, root, character_count=1):
        #: The player instance that will walk through the story
        self.player = player
        #: The root node of the story, i.e the starting node
        self.root = root    
        #: The number of different characters in the story (not currently used)
        self.character_count = character_count

    def getNextNode(self, current, speech):
        '''
        Compares the speech input to the keys for each child edge
        Attempts to find the closest match and return it.
        If no match is sufficently close, returns the 'current' node it was passed
        '''

        filler_words = ['go', 'to', 'the', 'a', 'an', 'for', 'if', 'not']

        # Check if user says exactly the edge's key
        for child in current.children:
            if speech.lower() == child.key.lower():
                return child

        speech = list(map(lambda i: stemmer.stem(i), speech.lower().split()))

        if "quit" in speech:
            return None

        for child in current.children:
            child_stems = list(
                map(lambda i: stemmer.stem(i), child.key.lower().split()))

            important_words = 0
            match_count = 0
            for stem in child_stems:
                if stem in filler_words:  # Disregard common words that carry no meaning
                    continue

                important_words += 1
                if stem in speech:
                    match_count += 1

            if match_count > important_words * .8:  # If the user said at least 80% of distinguishing
                                              # words in current.child's key
                return child

        return current

    def prereqsValid(self, player, newCurrent):
        for prereq in newCurrent.prereqs:
            if not prereq in player.completed.keys() or player.completed[prereq] == False:
                speak("You do not have your " + prereq +
                      " yet! Choose somewhere else to go.")
                return False
        return True

    def nextNode(self, player, lili):
        '''
        Takes player and liliAvatar as arguments. Does the speach associated
        with the current node, then takes the appropriate action associated
        with the connected edges. Returns thte next node that the player moves to. 
        '''

        current = player.location
        display_lili = current.image_path is None


        if current.auto is True: 
            nextEdge = current.chooseRandEdge()
            return nextEdge.traverseEdge(lili, display_lili)

        # At this point, the player must choose thier course of action
        # List options and allow the player to choose

        options = self.listChildren(current)

        lili.speak("What would you like to do? You can " + options, display_lili=display_lili)

        nextKey = lili.listen()

        nextEdge = self.getNextNode(current, nextKey)

        while nextEdge is current:
            lili.speak("I'm sorry! I didn't catch that.", display_lili=display_lili)
            lili.speak("What would you like to do? You can" + options, display_lili=display_lili)
            
            nextKey = lili.listen()

            nextEdge = self.getNextNode(current, nextKey)

        return nextEdge.traverseEdge(lili, display_lili)

    def walk(self, player, lili):
        '''
        'Main' function for the story
        Walks through the story, prompting the user when appropriate
        Exits when player's location is one of the 'ending' nodes specified
        in the constructor
        '''

        # Play starting node
        player.location.playNode(lili)
        player.completed[player.location.name] = True

        while player.location.name is not None:
            player.location = self.nextNode(player, lili)
            player.location.playNode(lili)
            player.completed[player.location.name] = True
            time.sleep(3)

    def listChildren(self, current):
        '''
        Speaks the list of child edge names aloud
        '''
        retval = ''
        for i in range(len(current.children)):
            child = current.children[i]

            if (i == len(current.children) - 1) and len(current.children) > 1:
                retval += 'or '
            retval += child.key

            if i != len(current.children) - 1:
                retval += ', '
            else:
                retval += '.'

        return retval

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
            print c.child.name,
        print '\n'

        for c in current.children:
            self.printStoryHelper(c.child)
