# ros2_android_cpp

This is a proof of concept of a ROS2 android app developed with qt in c++

Everything almost worked out of the box - the only trouble being copying the shared libraries over to the android system, and working around the rmw layer not being able to find libraries it wanted to load dynamically


How to get this thing to work(using qtcreator):

1) build ros2 for android(I had good success with this, however opensplice and cyclone don't build)

2) source your ros2 ws, set fastrtps as dds layer, launch qtcreator

3) copy over the libraries to the build directory (android-build/libs/armeabi-v7a)

4) run project
