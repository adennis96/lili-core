from story_node import StoryNode
from story_edge import StoryEdge
from dialog import Dialog

# StoryNode(self, name, description=None, dialog=None,
#                 children=None, image_path=None, auto=False)

# StoryEdge(self, key, child=None, dialog=None, weight=0)

# Story(self, player, root, endings)

# Dialog(self, characterID=0, text=None, filepath=None)

entrance_text = ('Hello, welcome to the San Diego Zoo! '
                 'We have a bunch of great exhibits for you today. '
                 'Say the name of the animal you want to see to go there.')

entrance_dialog = [Dialog(text=entrance_text)]

entrance = StoryNode('entrance', entrance_dialog, 'Node representing the entrance to the zoo.',
                     image_path='sanDiegoZoo.jpg')

monkey_edge_text = ('You follow the map you picked up at the entrance '
                    'to the monkey enclosure. As you get close you pass a funnel cake stand. '
                    'The smell of fried dough is heavy in the air.')

monkey_edge_dialog = Dialog(text=monkey_edge_text)

monkey_node_text = ('Look at the monkeys!')

monkey_node_dialog = [Dialog(text=monkey_node_text)]

monkey_node = StoryNode('monkey', monkey_node_dialog, 'Node representing the monkey exhibit.',
                        image_path='monkey.gif')

monkey_edge = StoryEdge('go to the monkeys', monkey_node,
                        dialog=monkey_edge_dialog)

penguin_edge_text = ('You walk over to the penguin enclosure.')

penguin_edge_dialog = Dialog(text=penguin_edge_text)

penguin_node_text = ('Look at the cute penguins!')

penguin_node_dialog = [Dialog(text=penguin_node_text)]

penguin_node = StoryNode('penguin', penguin_node_dialog, 'Node representing the penguin exhibit.',
                         auto=True, image_path='penguin.gif')

penguin_edge = StoryEdge('go to the penguins', penguin_node,
                         dialog=penguin_edge_dialog)

penguin_feeding_text = ('Look! They are feeding the penguins!')

penguin_feeding_dialog = [Dialog(text=penguin_feeding_text)]

penguin_feeding_node = StoryNode('penguin_feeding', penguin_feeding_dialog,
                                 image_path='penguin_feeding.gif', auto=True)

penguin_feeding_edge = StoryEdge('watch the penguins get fed', dialog=Dialog(text="Let's watch the penguins get fed!"),
                                 child=penguin_feeding_node, weight=1)

post_penguin_node = StoryNode('post_penguin', image_path='penguin.gif')

post_penguin_edge = StoryEdge(
    'finished with the penguins', child=post_penguin_node, weight=1)

lion_edge_text = ('You walk over to the lion enclosure.')

lion_edge_dialog = Dialog(text=lion_edge_text)

lion_node_text = ('That lion must be hungry!')

lion_node_dialog = [Dialog(text=lion_node_text)]

lion_node = StoryNode('lion', lion_node_dialog, 'Node representing the lion exhibit.',
                      image_path='lion_tries_to_grab_baby.gif')

lion_edge = StoryEdge('go to the lions', lion_node,
                      dialog=lion_edge_dialog)

exit_edge_text = (
    'You leave the exhibits and walk back toward the parking lot.')

exit_edge_dialog = Dialog(text=exit_edge_text)

exit_node_text = ('We hope you enjoyed your visit to the San Diego Zoo! '
                  'Come again soon.')

exit_node_dialog = [Dialog(text=exit_edge_text)]

exit_node = StoryNode('exit', exit_node_dialog,
                      'Node representing exiting the zoo.')

exit_edge = StoryEdge('leave the zoo', exit_node,
                      dialog=exit_edge_dialog)

zoo_nodes = [entrance, monkey_node, lion_node, penguin_node, exit_node]
zoo_start = entrance
zoo_ends = [exit_node]

entrance.children = [monkey_edge, penguin_edge, lion_edge]
monkey_node.children = [penguin_edge, lion_edge, exit_edge]

post_penguin_node.children = [monkey_edge, lion_edge, exit_edge]
penguin_node.children = [penguin_feeding_edge, post_penguin_edge]
penguin_feeding_node.children = [post_penguin_edge]

lion_node.children = [monkey_edge, penguin_edge, exit_edge]
