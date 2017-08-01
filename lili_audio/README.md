# lili-audio

This package provides:
* audio input
* audio output
* speech recognition
* text to speech

## IMPORTANT NOTE:
As of 6-19-2017, the ROS `audio_capture` package has a bug which prevents
streaming audio in WAVE format. This bug has been fixed in the development
version of the package, but not in the most recent public release. Since
the `lili_audio` package relies on this feature, an up to date version of the
`audio_capture` program has been included here (compiled for x86_64). If you run
into this bug, simply replace `/opt/ros/kinetic/lib/audio_capture/audio_capture`
with `lili_audio/include/audio_capture`. If this does not work, then simply
recompile the `audio_capture` package from the
[source](https://github.com/ros-drivers/audio_common).
