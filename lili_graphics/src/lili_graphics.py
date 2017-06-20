#!/usr/bin/env python

import pyglet
import std_msgs
import gtk.gdk
import rospy
import sys

class LilyAnimation(pyglet.window.Window):
    def __init__(self, fullscreen = True):
        pyglet.window.Window.__init__(self, fullscreen = fullscreen)
        self.lily_sprite = None
	self.image_path = rospy.get_param('~image_path', '/home/lmb420/Pictures/lilyResources/')

	print "Window constructed. Reindexing image path."

        pyglet.resource.path = [self.image_path]
        pyglet.resource.reindex()

        self.set_location(0,0)

        #set window background color = r, g, b, alpha
        #each value goes from 0.0 to 1.0
        self.background_color = 0.815, 0.87, 1, 1
        pyglet.gl.glClearColor(*self.background_color)

	print "Calling init_lili."

        self.init_lili()

	self.sub = rospy.Subscriber('display', std_msgs.msg.String, self.display_handler)

    def init_lili(self):
	try:
            lily_idle = pyglet.resource.animation("lily_idle.gif")
            print "Idle gif loaded."
            self.lily_sprite = pyglet.sprite.Sprite(lily_idle)
            self.lily_sprite.set_position((gtk.gdk.screen_width() - self.lily_sprite.width) / 2, (gtk.gdk.screen_height() - self.lily_sprite.height) / 2)
            print "Sprite constucted"
            self.on_draw()
        except pyglet.resource.ResourceNotFoundException:
            print "Invalid Resource Path! Exiting"
            sys.exit(1)

    def on_draw(self):
        #print("Starting on_draw")
        self.clear()
        self.lily_sprite.draw()

    def on_deactivate(self):
        self.minimize()

    def display_handler(self, filename):
        try:
	    image = pyglet.resource.animation(self.image_path + filename)
            self.lily_sprite.image = image
            self.lily_sprite.set_position((gtk.gdk.screen_width() - self.lily_sprite.width) / 2, (gtk.gdk.screen_height() - self.lily_sprite.height) / 2)        
	    self.on_draw()
        except:
            print "Invalid Resource Path! Exiting"
            sys.exit(1)

def animation_server():
    rospy.init_node('lili_graphics')
    print "Constructing LilyAnimation object."
    window = LilyAnimation()
    pyglet.app.run()
    rospy.spin()

if __name__ == '__main__':
    animation_server();
