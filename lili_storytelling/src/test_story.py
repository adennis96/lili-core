from story_node import StoryNode
from story_edge import StoryEdge

# StoryNode(self, name, description=None, dialog=None,
#                 children=None, image_path=None, auto=False)

# StoryEdge(self, key, child=None, dialog=None, weight=0)

# Story(self, player, root, endings)

entrance_dialog = ('Hello, welcome to the San Diego Zoo! '
                   'We have a bunch of great exhibits for you today. '
                   'Say the name of the animal you want to see to go there.')

entrance = StoryNode('entrance', 'Node representing the entrance to the zoo.',
                     dialog=entrance_dialog, image_path='sanDiegoZoo.jpg')

monkey_edge_dialog = ('You follow the map you picked up at the entrance '
                      'to the monkey enclosure. As you get close you pass a funnel cake stand. '
                      'The smell of fried dough is heavy in the air.')

monkey_node_dialog = ('Look at the monkeys!')

monkey_node = StoryNode('monkey', 'Node representing the monkey exhibit.',
                        dialog=monkey_node_dialog, image_path='monkey.gif')

monkey_edge = StoryEdge('go to the monkeys', monkey_node,
                        dialog=monkey_edge_dialog)

penguin_edge_dialog = ('You walk over to the penguin enclosure.')

penguin_node_dialog = ('Look at the cute penguins!')

penguin_node = StoryNode('penguin', 'Node representing the penguin exhibit.',
                         dialog=penguin_node_dialog, auto=True, image_path='penguin.gif')

penguin_edge = StoryEdge('go to the penguins', penguin_node,
                         dialog=penguin_edge_dialog)

penguin_feeding_node = StoryNode('penguin_feeding', dialog='Look! They are feeding the penguins!',
                                 image_path='penguin_feeding.gif', auto=True)

penguin_feeding_edge = StoryEdge('watch the penguins get fed', dialog="Let's watch the penguins get fed!",
                                 child=penguin_feeding_node, weight=1)

post_penguin_node = StoryNode('post_penguin', image_path='penguin.gif')

post_penguin_edge = StoryEdge(
    'finished with the penguins', child=post_penguin_node, dialog="We're done at the penguins.", weight=1)

lion_edge_dialog = ('You walk over to the lion enclosure.')

lion_node_dialog = ('That lion must be hungry!')

lion_node = StoryNode('lion', 'Node representing the lion exhibit.',
                      dialog=lion_node_dialog, image_path='lion_tries_to_grab_baby.gif')

lion_edge = StoryEdge('go to the lions', lion_node,
                      dialog=lion_edge_dialog)

exit_edge_dialog = (
    'You leave the exhibits are walk back toward the parking lot.')

exit_node_dialog = ('We hope you enjoyed your visit to the San Diego Zoo! '
                    'Come again soon.')

exit_node = StoryNode('exit', 'Node representing exiting the zoo.',
                      dialog=exit_node_dialog)

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
