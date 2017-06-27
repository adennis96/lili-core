from story_edge import StoryEdge
import utility_funcs

class StoryNode:
    '''
    This class represents a distinct moment in an interactive story.
    It will have fields for a name, description, a piece of dialog,
    and child story edges, which represent player interactions.
    '''
    def __init__(self, name, description = None, dialog = None,
            children = None, image_path = None, auto = False):
        
        self.name = str(name) #: This is a unique name for the node
        self.description = str(description) #! Description for dev purposes

        #: The text to be read to the player upon reaching this node
        self.dialog = str(dialog) 
        
        #: Children StoryEdges to this node representing player options, i.e. edges in graph
        self.children = [] 

        if children != None:
            for child in children:
                if isinstance(child, StoryEdge):
                    self.children.append(child)
                else:
                    utility_funcs.eprint("Invalid Child passed to StoryNode initializer!")

        # Add additional activities to the node
        self.image_path = image_path #: Image Path specifies the path to the image that should be displayed
        self.auto = auto #: Specifies whether the next edge should be taken automatically

        #self.task # currently unimplemented prerequisite system 
        
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
        if len(self.children) == 0
            return None

        if len(self.children) == 1
            return self.children[0]

        total = self.totalWeight()
        r = random.uniform(0, total)
        count = 0

        for child in self.children
            if count + child.weight >= r
                return child
            upto += child.weight

        utility_funcs.eprint("Something went wrong in chooseRandEdge.")
        return None

    def totalWeight(self):
        '''
        Returns the total weight of all of the child edges
        If this node has no children (is a leaf node), returns -1
        '''
        if len(self.children) == 0
            return -1

        total = 0
        for child in children
            total += child.weight

        return total

    def addPrereq(self, prereq):
        '''
        Currently unimplemented 'Task' prerequisite system
        Returns self
        '''
        return self

    

