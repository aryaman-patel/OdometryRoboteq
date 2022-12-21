PROJECT(Odometry)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(mylibSRCS  odometry.cpp RoboteqDevice.cpp)

ADD_LIBRARY(mylib SHARED ${mylibSRCS})
