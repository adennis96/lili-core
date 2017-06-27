
class StoryEdge:
	'''
	This class represents a possible path the player can take from a node.
	It is implemented as a directed edge on a digraph. It also has an optional
	dialog function and a bool that specifies the Edge should be taken auto-
	matically.
	It also contains the unimplemented Prerequisite functionality
	'''

	def __init__(self, key, parent = None, child = None, dialog = None, weight = 0):

		#: Key represents the string the player will say to take the node (irrelivant if auto is True in parent node)
		self.key = str(key)

		#: Dialog represents the string the player will be read when they traverse this edge
		self.dialog = str(dialog)

		self.parent = parent #: Parent node of this edge
		self.child = child #: Child node of this edge

		#: Weight represents the relative probablilty of this edge occuring if auto is set in the parent node
		self.weight = weight 