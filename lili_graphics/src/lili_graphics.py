#!/usr/bin/env python

import pyglet
import std_msgs
import rospy

class LiliAnimation(pyglet.window.Window):
    def __init__(self, fullscreen):
        pyglet.window.Window.__init__(self, fullscreen=fullscreen)
        self.image_path = rospy.get_param('~image_path')
        self.default_image = rospy.get_param('~default_image')

        pyglet.resource.path = [self.image_path]
        pyglet.resource.reindex()

        self.set_location(0,0)

        #set window background color = r, g, b, alpha
        #each value goes from 0.0 to 1.0
        self.background_color = 0.815, 0.87, 1, 1
        pyglet.gl.glClearColor(*self.background_color)

        self.sprite = None
        self.change_sprite(self.default_image)

        self.sub = rospy.Subscriber('display', std_msgs.msg.String,
            self.display_handler)

    def on_draw(self):
        self.clear()
        self.sprite.draw()

    def on_deactivate(self):
        if self.fullscreen:
            self.minimize()

    def display_handler(self, data):
        pyglet.clock.schedule_once(lambda x: self.change_sprite(data.data), 0)

    def change_sprite(self, filename):
        try:
            image = pyglet.resource.animation(filename)
        except pyglet.resource.ResourceNotFoundException:
            rospy.logerr("invalid resource path: " + filename)
        else:
            rospy.loginfo("image loaded: " + filename)
            if self.sprite == None:
                self.sprite = pyglet.sprite.Sprite(image)
            else:
                self.sprite.image = image
            sprite_x = (self.width - self.sprite.width) / 2
            sprite_y = (self.height - self.sprite.height) / 2
            self.sprite.set_position(sprite_x, sprite_y)

def animation_server():
    rospy.init_node('lili_graphics')
    fullscreen = rospy.get_param('~fullscreen', True)
    window = LiliAnimation(fullscreen)
    pyglet.app.run()

if __name__ == '__main__':
    animation_server();
