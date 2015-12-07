# Dron employee ROS package

## ROS interface

### Publish

* `/dron_employee/ready` :: **std_msgs/bool** - dron is ready to start
* `/dron_employee/wifi` :: **dron_employee/Wireless** - WiFi ESSID and password for client connection
* `/dron_employee/video` :: **std_msgs/string** - link to recorded video

### Subscribe

* `/dron_employee/target` :: **dron_employee/Target** - client position and roadmap 

