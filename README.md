
# Odometry Roboteq
<p align="center">
 <img src="https://user-images.githubusercontent.com/117113574/218284939-c0ef72ca-89f9-40f9-89da-f1270b33150c.png">
 </br>


  <img src="https://user-images.githubusercontent.com/117113574/218284932-63f40b05-c00b-47c2-b315-7f255deb7269.png">
</p>


This project is a implementation of Odometry calculations from wheel ecncoders of a Roboteq Motor.
The API in use is proveded by [Roboteq](https://www.roboteq.com/products/software/apis-drivers).

The source code is in the `odometry.cpp` file, along with that I have also written a python wrapper `wrapper_py.py` for this 
cpp file for easier integration. 

The implementation also, includes a function to make the robot translate by a distance and rotate by an angle input
the `odometry.py` file has the function `dist_corr()` and `angle_corr()` that take in distance and 
angle input along with the direction of rotation from motors. 

Further understanding the use of encoder wraparound used in the source file for odometry calculations 
look at [differential_drive_pkg](http://wiki.ros.org/differential_drive) from ROS which has the 
explanation for `wheel_low_wrap` and `wheel_high_wrap`.








### Additional resouce:
https://answers.ros.org/question/224123/integer-overflow-in-differential_drive-wheel-encoders/
