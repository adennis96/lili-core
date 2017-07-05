from story_node import StoryNode
from story_edge import StoryEdge
from dialog import Dialog

# StoryNode(self, name, description=None, dialog=None,
#                 children=None, image_path=None, auto=False)

# StoryEdge(self, key, child=None, dialog=None, weight=0)

# Story(self, player, root)

# Dialog(self, characterID=0, text=None, filepath=None)


### Entrance ###
entrance_text = ('Hello, welcome to the San Diego Zoo! '
                 'We have a bunch of great exhibits for you today. '
                 'Say the name of the animal you want to see to go there.')

entrance_dialog = [Dialog(text=entrance_text)]

entrance = StoryNode('entrance', entrance_dialog, 'Node representing the entrance to the zoo.',
                     image_path='sanDiegoZoo.jpg')


### Monkeys ###
monkey_edge_text = ('You walk to the monkey enclosure. As you get close you pass a funnel cake stand. '
                    'The smell of fried dough is heavy in the air.')

monkey_edge_dialog = Dialog(text=monkey_edge_text)

monkey_node_dialog = [Dialog(text='Look at the monkeys!')]

monkey_node = StoryNode('monkey', monkey_node_dialog, 'Node representing the monkey exhibit.',
                        image_path='monkey.gif')

monkey_edge = StoryEdge('go to the monkeys', monkey_node,
                        dialog=monkey_edge_dialog)


### Funnel Cake! ###
funnel_edge_text = ('You walk to the funnel cake stand and hop on the end of the line.')

funnel_node_dialog = [Dialog(text='As you wait in line, the smell of funnel cake grows stronger.'),
                      Dialog(text='Finally you get to the front and buy your cake.'),
                      Dialog(text='You find a place to sit down and enjoy your sugary treat.')]

funnel_node = StoryNode('funnel', funnel_node_dialog, 'Node representing the funnel cake stand',
                        image_path='funnelCake.jpg')

funnel_edge = StoryEdge('buy a funnel cake', funnel_node,
                        dialog=funnel_edge_text)


### Penguins ###
penguin_edge_dialog = Dialog(text='You walk over to the penguin enclosure.')

penguin_node_dialog = [Dialog(text='Look at the cute penguins!')]

penguin_node = StoryNode('penguin', penguin_node_dialog, 'Node representing the penguin exhibit.',
                         auto=True, image_path='penguin.gif')

penguin_edge = StoryEdge('go to the penguins', penguin_node,
                         dialog=penguin_edge_dialog)


### Penguin Feeding ###
penguin_feeding_dialog = [Dialog(text="Let's stay and watch!")]

penguin_feeding_node = StoryNode('penguin_feeding', penguin_feeding_dialog,
                                 image_path='penguin_feeding.gif', auto=True)

penguin_feeding_edge = StoryEdge('watch the penguins get fed', dialog='Great Timing! They are feeding the penguins!',
                                 child=penguin_feeding_node, weight=1)

post_penguin_node = StoryNode('post_penguin', image_path='penguin.gif')

post_penguin_edge = StoryEdge(
    'finished with the penguins', child=post_penguin_node, weight=1)


### Lions ###
lion_edge_dialog = Dialog(text='You walk over to the lion enclosure.')

lion_node_dialog = [Dialog(text='That lion must be hungry!'), Dialog(text='Listen to it roar!'),
                    Dialog(filepath='lion.wav')]

lion_node = StoryNode('lion', lion_node_dialog, 'Node representing the lion exhibit.',
                      image_path='lion_tries_to_grab_baby.gif')

lion_edge = StoryEdge('go to the lions', lion_node,
                      dialog=lion_edge_dialog)


### Elephant ###
elephant_node_dialog = [Dialog(text='Elephants are my favorite!'),
                        Dialog(text='The elephant says Hi...'),
                        Dialog(filepath='elephant.wav')]

elephant_node = StoryNode('elephant', elephant_node_dialog, 'Node representing the elephant exhibit.',
                            image_path='elephant.gif', auto=True)

elephant_edge = StoryEdge('go to the elephants', elephant_node,
                            dialog='You walk over to the elephant exhibit.')


### Elephant Painting
elephant_painting_dialog = [Dialog(text='Look at the beautiful painting!')]

elephant_painting_node = StoryNode('elephant_painting', elephant_painting_dialog, 'Node with elephant painting', 
                            image_path='elephant_painting.gif', auto=True)

elephant_painting_edge = StoryEdge('elephant painting', elephant_painting_node, 
                            dialog='How lucky! One of the elephants is going to paint for us!', weight=2)

post_elephant_node = StoryNode('post_elephant', image_path='elephant.gif')

post_elephant_edge = StoryEdge('done at the elephants', post_elephant_node, weight=1)



### Exit ###
exit_edge_dialog = Dialog(text='You leave the exhibits and walk back toward the parking lot.')

exit_node_text = ('We hope you enjoyed your visit to the San Diego Zoo! '
                  'Come again soon.')

exit_node_dialog = [Dialog(text=exit_node_text)]

exit_node = StoryNode(None, exit_node_dialog,
                      'Node representing exiting the zoo.')

exit_edge = StoryEdge('leave the zoo', exit_node,
                      dialog=exit_edge_dialog)


# For the Story construction:
zoo_nodes = [entrance, monkey_node, lion_node, penguin_node, funnel_node,
              penguin_feeding_node, exit_node, post_penguin_node, elephant_node]
zoo_start = lion_node


# Story Structure:

entrance.children = [monkey_edge, penguin_edge, lion_edge, elephant_edge]

monkey_node.children = [penguin_edge, lion_edge, elephant_edge, funnel_edge, exit_edge]

funnel_node.children = [lion_edge, penguin_edge, elephant_edge, exit_edge]

post_penguin_node.children = [monkey_edge, lion_edge, elephant_edge, exit_edge]
penguin_node.children = [penguin_feeding_edge, post_penguin_edge]
penguin_feeding_node.children = [post_penguin_edge]

lion_node.children = [monkey_edge, penguin_edge, elephant_edge, exit_edge]

post_elephant_node.children = [penguin_edge, lion_edge, monkey_edge, exit_edge]
elephant_node.children = [elephant_painting_edge, post_elephant_edge]
elephant_painting_node.children = [post_elephant_node]