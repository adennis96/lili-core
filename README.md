# lili-core

These ROS packages provide implementation for LILI, an affordable robot designed
to socialize with children with autism.

## Getting started

1. Install [ROS (the Robot Operating System)](http://www.ros.org)
2. Create a workspace and clone the repository
   ```
   mkdir workspace
   cd workspace
   git clone https://github.com/adennis96/lili-core.git
   mv lili-core src
   ```
3. Make the package: `catkin make`
4. Source the workspace: `source devel/setup.bash`
   (Note: this must be run on each new terminal or else added to ~/.bashrc)

## Package Descriptions

### lili_audio
provides audio input and output as well as speech recognition and text to speech
### lili_bringup
provides ROS drivers for hardware components
### lili_description
provides a description of the physical configuration of the robot (eg. distance
from one hardware component to another, 3d renderings of hardware components) to
be used by software
### lili_graphics
displays images and gifs to the screen
### lili_navigation
allows the robot to map its environment and move around obstacles
### lili_storytelling
allows the robot to tell pre-made interactive stories
### lili_vision
allows the robot to detect objects in its field of view
