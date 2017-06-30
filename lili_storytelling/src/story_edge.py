
class StoryEdge:
    '''
    This class represents a possible path the player can take from a node.
    It is implemented as a directed edge on a digraph. It also has an optional
    dialog function and a bool that specifies the Edge should be taken auto-
    matically.
    It also contains the unimplemented Prerequisite functionality
    '''

    def __init__(self, key, child=None, dialog=None, weight=0):

        #: Key represents the string the player will say to take the node (irrelivant if auto is True in parent node)
        self.key = str(key)

        #: A Dialog object that represents the text/sounds that will be played when the player traverses this node
        self.dialog = dialog

        self.child = child  # : Child node of this edge

        #: Weight represents the relative probablilty of this edge occuring if auto is set in the parent node
        self.weight = weight

    def traverseEdge(self, lili, display_lili=True):
        '''
        Performs all actions associated with traversing an edge in the story
        i.e. speak the associated dialog
        Returns the child node
        '''

        if self.dialog is not None:
            lili.speak(self.dialog, block=True, display_lili=display_lili)

        return self.child
