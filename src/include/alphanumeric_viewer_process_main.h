/*!********************************************************************************
 * \brief     Alphanumeric Viewer implementation 
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#ifndef ALPHANUMERIC_VIEWER_ROSMODULE_H
#define ALPHANUMERIC_VIEWER_ROSMODULE_H

//STD CONSOLE
#include <sstream>
#include <stdio.h>
#include <curses.h>
#include <iostream>
#include <string>
#include <math.h> 

//ROS
#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Imu.h"
#include "aerostack_msgs/FlightState.h"
#include "aerostack_msgs/SetControlMode.h"
#include "aerostack_msgs/MotionControlMode.h"
#include "mav_msgs/RollPitchYawrateThrust.h" 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define FREQ_INTERFACE 8.0
//MsgsROS
sensor_msgs::BatteryState battery_msg;
geometry_msgs::PointStamped altitude_msg;
geometry_msgs::PointStamped altitude_sea_level_msg;
geometry_msgs::TwistStamped ground_speed_msg;
geometry_msgs::TwistStamped current_speed;
sensor_msgs::Imu imu_msg;
sensor_msgs::Temperature temperature_msg;
aerostack_msgs::FlightState quadrotor_status_msg;
geometry_msgs::PoseStamped current_pose;;
geometry_msgs::PoseStamped current_position_reference;
geometry_msgs::TwistStamped current_speed_reference;
geometry_msgs::PoseStamped actuator_command_roll_pitch_msg;
geometry_msgs::TwistStamped actuator_command_altitude_yaw_msg;
mav_msgs::RollPitchYawrateThrust thrust_msg;

int last_received_control_mode;
double r, p, yaw;

//Subscribers
ros::Subscriber self_localization_pose_sub;
ros::Subscriber self_localization_speed_sub;
ros::Subscriber battery_sub;
ros::Subscriber imu_sub;
ros::Subscriber altitude_sub;
ros::Subscriber altitude_sea_level_sub;
ros::Subscriber ground_speed_sub;
ros::Subscriber temperature_sub;
ros::Subscriber status_sub;
ros::Subscriber actuator_command_roll_pitch_sub;
ros::Subscriber actuator_command_altitude_yaw_sub;
ros::Subscriber control_mode_sub;
ros::Subscriber position_reference_subscriber;
ros::Subscriber speed_reference_subscriber;
ros::Subscriber thrust_sub;
    
//Variables
std::stringstream interface_printout_stream;
std::stringstream pinterface_printout_stream;
std::string drone_id_namespace;

//Aux (check if a variable has been received)
bool battery_aux;
bool altitude_aux;
bool altitude_sea_level_aux;
bool ground_speed_aux;
bool imu_aux;
bool temperature_aux;
bool current_speed_reference_aux;
bool current_position_reference_aux;
bool actuator_command_altitude_yaw_aux;
bool actuator_command_roll_pitch_aux;  
bool current_pose_aux;
bool current_speed_aux;
bool thrust_aux;

//Topics 
std::string battery_topic_name;
std::string altitude_topic_name;
std::string altitude_sea_level_topic_name;
std::string self_localization_speed_topic_name;
std::string ground_speed_topic_name;
std::string temperature_topic_name;
std::string imu_topic_name;
std::string actuator_command_roll_pitch_topic_name;
std::string actuator_command_altitude_yaw_topic_name;
std::string assumed_control_mode_topic_name;
std::string status_topic_name;
std::string self_localization_pose_topic_name;
std::string motion_reference_speed_topic_name;
std::string motion_reference_pose_topic_name;
std::string actuator_command_thrust_topic_name;

//Print-Stream Functions
void printStream(float var, bool aux);
void printStream(double var, bool aux);
void printStream3(float var,bool aux);
void printQuadrotorState();
void printControlMode();
void printBattery();
void printSensorMenu();
void printNavigationMenu();
void printSensorValues();
void printNavigationValues();

//Callback Functions
void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
void selfLocalizationSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);
void altitudeCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
void altitudeSeaLevelCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
void groundSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void droneStatusCallback(const aerostack_msgs::FlightState::ConstPtr& msg);
void actuatorCommandRollPitchCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void actuatorCommandAltitudeYawCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
void positionRefsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
void speedRefsSubCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
void controlModeSubCallback(const aerostack_msgs::MotionControlMode::ConstPtr &msg);
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
void temperatureCallback(const sensor_msgs::Temperature::ConstPtr &msg);
void thrustSubCallback(const mav_msgs::RollPitchYawrateThrust::ConstPtr &msg);

#endif 


