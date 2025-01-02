# sensor_interface_over_tcp

## in the folder src/sensor_interface there are two nodes
## The sensor_server node where the sensor acts as the TCP server using the given communication protocol
## The sensor_client node where the controller communicates with the sensor, this node automatically starts the start command with the default interval of 1000 milliseconds, the interval can be adjusted at the start of the node such as 
## ros2 run sensor_interface sensor_client --ros-args -p interval:=2000 "Here the interval can be set to desired values"
## By starting both nodes we can see the communication between the  controller and sensor and the sensor and controller
## Also we can start, or stop the communication with the help of service calls, where stop_command service is used to stop the communication and start_command service is used to start again
## Also we can set the interval value by the "ros2 param set" command to the desired interval and starting again the communication with the service call will start with the changes made to interval
## For user to control the communication, have made an user friendly GUI with the help of Python tkinter library which was easy to implement 
## For using the GUI we have to run the python file user_gui.py, which is located at src/sensor_interface/sensor_interface

## A video is Attached which shows the communication made by sensor controller ,also the user GUI




<video width="600" controls>
  <source src="Screencast from 2025-01-02 01-11-44.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>
