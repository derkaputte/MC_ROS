# MC_ROS
Arduino firm for ROS and MC33926

This simple firmware works for my Arduino MEGA.
It subscribe following ROS-topics: "speed_l" and "speed_r".
Also read the ticks of the left motor encoder and publishs it to the topic: "ticks_left".

You have to setup a ROS-node of rosserial. In this case it works fine with :
rosrun rosserial_python serial_node.py /dev/ttyUSB0 (or whatever your mc is connected)


