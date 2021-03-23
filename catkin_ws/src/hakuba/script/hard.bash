roscore

ypspur-coordinator -p /usr/local/share/robot-params/speego.param -d /dev/ttyACM0

rosrun ypspur_ros_bridge ypspur_ros_bridge

rosrun urg_node urg_node _serial_port:=/dev/ttyACM1