from story_edge import StoryEdge
import utility_funcs
import random

class StoryNode:
    '''
    This class represents a distinct moment in an interactive story.
    It will have fields for a name, description, a piece of dialog,
    and child story edges, which represent player interactions.
    '''

    def __init__(self, name, dialogs = [], description=None,
                 children=None, image_path=None, auto=False):

        if name is not None:
            self.name = str(name)  #: This is a unique name for the node
        else:
            self.name = None
        
        if description is not None:
            self.description = str(description)  #: Description for dev purposes
        else:
            self.description = None

        #: A list of Dialog objects containing text and associated characters
        self.dialogs = dialogs

        #: Children StoryEdges to this node representing player options, i.e. edges in graph
        self.children = []

        if children != None:
            for child in children:
                if isinstance(child, StoryEdge):
                    self.children.append(child)
                else:
                    utility_funcs.eprint(
                        "Invalid Child passed to StoryNode initializer!")

        # Add additional activities to the node
        #: Image Path specifies the path to the image that should be displayed
        self.image_path = image_path
        self.auto = auto  #: Specifies whether the next edge should be taken automatically

        # self.task # currently unimplemented prerequisite system

    def addChild(self, child):
        '''
        Appends StoryEdge child to the list of children
        Returns self
        '''
        if isinstance(child, StoryEdge):
            self.children.append(child)
        else:
            utility_funcs.eprint("Invalid Child passed to StoryNode addChild!")
        return self

    def chooseRandEdge(self):
        '''
        Chooses and returns a random connected edge based on weighting
        Returns None if this node contains no child edges
        '''
        if len(self.children) == 0:
            return None

        if len(self.children) == 1:
            return self.children[0]

        total = self.totalWeight()
        utility_funcs.eprint(str(total) + " is the total weight for this branch.")

        # If children has multiple member all with weight 0
        if total is 0:
            r = ramdom.randrange(len(self.children))
            return self.children[r]

        r = random.uniform(0, total)
        count = 0

        for child in self.children:
            if (count + child.weight) >= r:
                return child
            count += child.weight

        utility_funcs.eprint("Something went wrong in chooseRandEdge.")
        return None

    def playNode(self, lili):
        '''
        Performs all actions associated with 'playing a node'
        i.e. displaying the associated image and speaking the dialog aloud
        Returns self
        '''
        if self.image_path is not None:
            lili.display(self.image_path)
        else:
            lili.display()

        for dialog in self.dialogs:
            # TODO add support for multiple characters using dialog.characterID
            lili.speak(dialog, block=True, display_lili=self.image_path is None)

        return self

    def totalWeight(self):
        '''
        Returns the total weight of all of the child edges
        If this node has no children (is a leaf node), returns -1
        '''
        if len(self.children) == 0:
            return -1

        total = 0
        for child in self.children:
            total += child.weight

        return total

    def addPrereq(self, prereq):
        '''
        Currently unimplemented 'Task' prerequisite system
        Returns self
        '''
        return self
