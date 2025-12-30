# Extreme 3d Pro Interfacing
The package required to control the joystick is 'teleop_twist_joy'

## extreme_3d_pro_control package
Clone the repo and build the workspace. 

Add the **e3dp.config.yaml** file to **/opt/ros/{ros_distro}/share/teleop_twist_joy/config**.

The parameters of the yaml file can be changed according to need. The default required_enable_button parameter is set to false but if it is changed to true, then uncomment the following two parameters. To use the controller to publish 
messages, hold button 2 and to operate in turbo mode, hold button 12.

## Launching
In a terminal, run:
```
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='e3dp' 
```
In another terminal, run:
```
ros2 run extreme_3d_pro_control controller
```

## Web UI
In your browser, type **http://localhost:8000**.

To access from any other device within the same Wi-fi, type **http://ip_address:8000** where the ip address can be found by running the command:
```
hostname -I
```

## TwistStamped messages
The default message published to /cmd_vel by the launch file is Twist type.

To make it TwistStamped, in the controller.py, uncomment the twist_stamped_sub subscriber and the twist_stamped_callback function while commenting the ones for the twist type message.

Modify the launch file command line arguments:
```
ros2 launch teleop_twist_joy teleop-launch.py   joy_config:='e3dp' publish_stamped_twist:='true'
```
