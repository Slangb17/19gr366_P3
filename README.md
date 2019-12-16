# 19gr366_P3 ROB3

The folder ***'crustcrawler_ino'*** contains the Arduino Mega program for controlling the CrustCrawler with either the Myo Armband or joysticks as an input device.

The folder ***'myo_armband'*** contains the C++ program utilized to connect to the Myo Armband and extract gestures and orientation data. Furthermore, the program establises a serial communication to the Arduino Mega for sending information.

The folder ***dynamics_control_system*** contains the *MATLAB* script (requires [Robotics Toolbox](http://petercorke.com/wordpress/toolboxes/robotics-toolbox)) used to compute the dynamics for the CrustCrawler. Additionally, a computed-torque controller made in *simulink* can also be found in the folder.


