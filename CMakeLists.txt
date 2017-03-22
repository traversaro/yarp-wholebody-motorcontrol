# Copyright: 2017 iCub Facility, Istituto Italiano di Tecnologia
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the MIT Licence
# 

cmake_minimum_required(VERSION 2.8.11)
project(yarpWholeBodyMotorControlExample)

find_package(YARP 2.3.68 REQUIRED)

add_executable(yarpWholeBodyMotorControlExample yarpWholeBodyMotorControlExample.cpp)
target_include_directories(yarpWholeBodyMotorControlExample PRIVATE ${YARP_INCLUDE_DIRS})
target_link_libraries(yarpWholeBodyMotorControlExample ${YARP_LIBRARIES})