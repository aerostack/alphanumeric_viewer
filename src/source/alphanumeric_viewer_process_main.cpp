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

#include "../include/alphanumeric_viewer_process_main.h"

int main(int argc, char **argv)
{
    //OUTPUT FORMAT
    interface_printout_stream << std::fixed << std::setprecision(2) << std::setfill('0');

    //ROS 
    ros::init(argc, argv, "alphanumeric_viewer_process");
    ros::NodeHandle n("~");

    //Configuration
    n.param<std::string>("drone_id_namespace", drone_id_namespace, "drone1");

    ros::param::get("~battery_topic_name", battery_topic_name);
    ros::param::get("~altitude_topic_name", altitude_topic_name);
    ros::param::get("~altitude_sea_level_topic_name", altitude_sea_level_topic_name);
    ros::param::get("~ground_speed_topic_name", ground_speed_topic_name);
    ros::param::get("~imu_topic_name", imu_topic_name);
    ros::param::get("~actuator_command_roll_pitch_topic_name", actuator_command_roll_pitch_topic_name);
    ros::param::get("~actuator_command_altitude_yaw_topic_name", actuator_command_altitude_yaw_topic_name);
    ros::param::get("~assumed_control_mode_topic_name", assumed_control_mode_topic_name);
    ros::param::get("~status_topic_name", status_topic_name);
    ros::param::get("~temperature_topic_name", temperature_topic_name);
    ros::param::get("~self_localization_speed_topic_name",self_localization_speed_topic_name);
    ros::param::get("~self_localization_pose_topic_name", self_localization_pose_topic_name);
    ros::param::get("~motion_reference_speed_topic_name", motion_reference_speed_topic_name);
    ros::param::get("~motion_reference_pose_topic_name", motion_reference_pose_topic_name);
    ros::param::get("~actuator_command_thrust_topic_name", actuator_command_thrust_topic_name);

    //Sensor measurements subscribers
    battery_sub=n.subscribe("/"+drone_id_namespace+"/"+battery_topic_name, 1, &batteryCallback);
    altitude_sub=n.subscribe("/"+drone_id_namespace+"/"+altitude_topic_name, 1, &altitudeCallback);
    altitude_sea_level_sub=n.subscribe("/"+drone_id_namespace+"/"+altitude_sea_level_topic_name, 1, &altitudeSeaLevelCallback);
    ground_speed_sub=n.subscribe("/"+drone_id_namespace+"/"+ground_speed_topic_name, 1, &groundSpeedCallback);
    imu_sub=n.subscribe("/"+drone_id_namespace+"/"+imu_topic_name, 1, &imuCallback);
    temperature_sub = n.subscribe("/"+drone_id_namespace+"/"+temperature_topic_name, 1, &temperatureCallback);

    //Actuator commands subscribers
    actuator_command_roll_pitch_sub = n.subscribe("/"+drone_id_namespace+"/"+actuator_command_roll_pitch_topic_name, 1, &actuatorCommandRollPitchCallback);
    actuator_command_altitude_yaw_sub = n.subscribe("/"+drone_id_namespace+"/"+actuator_command_altitude_yaw_topic_name, 1, &actuatorCommandAltitudeYawCallback);
    control_mode_sub=n.subscribe(std::string("/"+drone_id_namespace + "/" +assumed_control_mode_topic_name), 1, &controlModeSubCallback);
    thrust_sub=n.subscribe(std::string("/"+drone_id_namespace + "/" +actuator_command_thrust_topic_name), 1, &thrustSubCallback);

    //Self localization subscriber
    self_localization_pose_sub= n.subscribe("/"+drone_id_namespace+"/"+self_localization_pose_topic_name, 1, &selfLocalizationPoseCallback);
    self_localization_speed_sub= n.subscribe("/"+drone_id_namespace+"/"+self_localization_speed_topic_name, 1, &selfLocalizationSpeedCallback);
    status_sub=n.subscribe("/"+drone_id_namespace+"/"+status_topic_name, 1, &droneStatusCallback);
    
    //Motion references subscriber
    position_reference_subscriber= n.subscribe("/"+drone_id_namespace+"/"+motion_reference_pose_topic_name, 1, &positionRefsCallback);
    speed_reference_subscriber= n.subscribe("/"+drone_id_namespace+"/"+motion_reference_speed_topic_name, 1, &speedRefsSubCallback);
    
    //ncurses initialization (output text)
    initscr();
    start_color();
    use_default_colors();  
    curs_set(0);
    noecho();
    nodelay(stdscr, TRUE);
    erase();
    refresh();
    init_pair(1, COLOR_GREEN, -1);
    init_pair(2, COLOR_RED, -1);
    init_pair(3, COLOR_YELLOW, -1);
    init_pair(4, COLOR_CYAN, -1);

    printSensorMenu();
       
    //Rate
    ros::Rate loop_rate(FREQ_INTERFACE);
    char command = 0;

    // 0 Sensor
    // 1 Navigation
    int window = 0;

    //Aux Sensors
    battery_aux = false;
    altitude_aux = false;
    altitude_sea_level_aux = false;
    ground_speed_aux = false;
    imu_aux = false;
    temperature_aux = false;

    //Aux navigation
    current_speed_reference_aux = false;
    current_position_reference_aux = false;
    actuator_command_altitude_yaw_aux = false;
    actuator_command_roll_pitch_aux = false;  
    current_pose_aux = false;
    current_speed_aux = false;
    thrust_aux = false;

    //Loop
    while (ros::ok()) {
        //Read messages
        ros::spinOnce();

        //Get key
        command = getch();
        switch (command){
            case 'S':
            case 's':  // Sensor
                erase();
                refresh();
                printSensorMenu();
                window = 0;
            break;
            case 'N':
            case 'n':  // Navigation
                erase();
                refresh();
                printNavigationMenu();
                window = 1;
            break;
        }

        //Print values
        switch (window){
            case 0:
                printSensorValues();
            break;
            case 1:
                printNavigationValues();
            break;
        }

        //Refresh
        refresh();
        loop_rate.sleep();
    }

    //End ncurses
    endwin();
    return 0;
}

//Print battery charge
void printBattery(){
    if(battery_aux){
        interface_printout_stream << std::fixed << std::setprecision(0) << std::setfill(' ');
        interface_printout_stream.clear();
        interface_printout_stream.str(std::string());
        clrtoeol(); refresh();
        float percentage = battery_msg.percentage * 100;
        interface_printout_stream << std::setw(2) << std::internal << percentage;
        if(battery_msg.percentage == 1) {
            attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
        }
        if(battery_msg.percentage > 0.5 && battery_msg.percentage < 1) {
            attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
        }
        if(battery_msg.percentage <= 0.5 && battery_msg.percentage > 0.2) {
            attron(COLOR_PAIR(3));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(3));
        }
        if(battery_msg.percentage <= 0.2) {
            attron(COLOR_PAIR(2));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(2));  
        }
    }else{ //Battery has not been received
        printw("---");        
    }
    printw(" %%");
    interface_printout_stream << std::fixed << std::setprecision(2) << std::setfill('0');
}

//Print float using stringstream
void printStream(float var,bool aux) {
    if(aux){
        interface_printout_stream.clear();
        interface_printout_stream.str(std::string());
        if (var > -0.01){
            interface_printout_stream << std::setw(5) << std::internal << fabs(var);
            attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
        }else{
            interface_printout_stream << std::setw(6) << std::internal << var;
            attron(COLOR_PAIR(2));printw("%s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(2));
        }
    }else{
        printw("--.--");
    }
} 

//Print float using stringstream with 3 units
void printStream3(float var,bool aux) {
    if(aux){
        interface_printout_stream.clear();
        interface_printout_stream.str(std::string());
        if (var > -0.01){
            interface_printout_stream << std::setw(6) << std::internal << fabs(var);
            attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
        }else{
            interface_printout_stream << std::setw(7) << std::internal << var;
            attron(COLOR_PAIR(2));printw("%s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(2));
        }
    }else{
        printw("---.--");
    }
} 

//Print double using stringstream
void printStream(double var,bool aux) {
    if(aux){
        interface_printout_stream.clear();
        interface_printout_stream.str(std::string());
        if (var > -0.01){
            interface_printout_stream << std::setw(5) << std::internal << fabs(var);
            attron(COLOR_PAIR(1));printw(" %s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(1));
        }else{
            interface_printout_stream << std::setw(6) << std::internal << var;
            attron(COLOR_PAIR(2));printw("%s",interface_printout_stream.str().c_str());attroff(COLOR_PAIR(2));
        }
    }else{
        printw("--.--");
    }
} 

//Print control mode
void printControlMode(){
    switch (last_received_control_mode) {
    case aerostack_msgs::MotionControlMode::SPEED:
        printw("SPEED        ");
        break;
    case aerostack_msgs::MotionControlMode::POSE:
        printw("POSITION     ");
        break;
    case aerostack_msgs::MotionControlMode::GROUND_SPEED:
        printw("GROUND SPEED ");
        break;
    case aerostack_msgs::MotionControlMode::SPEED_3D:
        printw("SPEED 3D     ");
        break; 
    case aerostack_msgs::MotionControlMode::ATTITUDE:
        printw("ATTITUDE     ");
        break;
    case aerostack_msgs::MotionControlMode::TRAJECTORY:
        printw("TRAJECTORY   ");
        break;                   
    case aerostack_msgs::MotionControlMode::UNKNOWN:
    default:
        printw("UNKNOWN      ");
        break;
    }
}

//Print status
void printQuadrotorState(){
    switch (quadrotor_status_msg.state) {
        case aerostack_msgs::FlightState::UNKNOWN:
            printw("UNKNOWN   ");
            break;
        case aerostack_msgs::FlightState::LANDED:
            printw("LANDED    ");
            break;
        case aerostack_msgs::FlightState::FLYING:
            printw("FLYING    ");
            break;
        case aerostack_msgs::FlightState::HOVERING:
            printw("HOVERING  ");
            break;
        case aerostack_msgs::FlightState::TAKING_OFF:
            printw("TAKING OFF");
            break;
        case aerostack_msgs::FlightState::LANDING:
            printw("LANDING   ");
            break;
    }
}
//Sensor window
void printSensorValues(){
    //DroneID
    move(4,4);
    attron(COLOR_PAIR(4));printw("%s",drone_id_namespace.c_str());attroff(COLOR_PAIR(4));
    //Battery
    move(6,4);
    printBattery();
    //Speed
    move(8,4);
    printStream(ground_speed_msg.twist.linear.x,ground_speed_aux);printw(",");
    move(8,11);
    printStream(ground_speed_msg.twist.linear.y,ground_speed_aux);printw(",");
    move(8,18);
    printStream(ground_speed_msg.twist.linear.z,ground_speed_aux);printw(" m/s   ");

    //Pose IMU
    tf2::Matrix3x3 imu_m(tf2::Quaternion (imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w));
    r = 0; p = 0; yaw = 0;
    imu_m.getRPY(r, p, yaw);
    if (std::isnan(r)) r = 0.0; 
    if (std::isnan(p)) p = 0.0; 
    if (std::isnan(yaw)) yaw = 0.0; 

    move(10,4);
    printStream(yaw,imu_aux);printw(",");
    move(10,11);
    printStream(p,imu_aux);printw(",");
    move(10,18);
    printStream(r,imu_aux);printw(" rad   ");

    //Speed IMU
    move(12,4);
    printStream(imu_msg.angular_velocity.z,imu_aux);printw(",");
    move(12,11);
    printStream(imu_msg.angular_velocity.y,imu_aux);printw(",");
    move(12,18);
    printStream(imu_msg.angular_velocity.x,imu_aux);printw(" rad/s  ");

    //Acceleration IMU
    move(14,4);
    printStream(imu_msg.linear_acceleration.x,imu_aux);printw(",");
    move(14,11);
    printStream(imu_msg.linear_acceleration.y,imu_aux);printw(",");
    move(14,18);
    printStream(imu_msg.linear_acceleration.z,imu_aux);printw(" m/s2   ");

    //Altitude
    move(4,52);
    printStream(altitude_msg.point.z,altitude_aux);printw(" m");
    //Altitude sea level
    move(6,52);
    printStream(altitude_sea_level_msg.point.z,altitude_sea_level_aux);printw(" m");
    //Temperature
    move(8,52);
    printStream(temperature_msg.temperature,temperature_aux);printw(" Degrees celsius");
}
void printSensorMenu(){
    move(0,0);
    printw("                - ALPHANUMERIC VIEWER OF AERIAL ROBOTICS DATA -");
    move(1,0);
    printw("                        Key: S (sensors), N (navigation)");
    move(2,0);
    printw("                             ^                          ");
    //Left column  
    move(3,0);
    printw(" Drone id:");
    move(5,0);
    printw(" Battery charge:");
    move(7,0);
    printw(" Speed (x,y,z):");
    move(9,0);
    printw(" Pose IMU (yaw,pitch,roll):");
    move(11,0);
    printw(" Speed IMU (yaw,pitch,roll):");
    move(13,0);
    printw(" Acceleration IMU (x,y,z):");

    //Right column
    move(3,50);
    printw("Altitude (-z):");
    move(5,50);
    printw("Altitude (sea level):");
    move(7,50);
    printw("Temperature:");
}

void printNavigationValues(){
    //Measurements
    move(5,14);
    printStream(altitude_msg.point.z,altitude_aux);printw(" m");
    //Speed
    move(6,14);
    printStream(ground_speed_msg.twist.linear.x,ground_speed_aux);printw(",");
    move(6,21);
    printStream(ground_speed_msg.twist.linear.y,ground_speed_aux);printw(",");
    move(6,28);
    printStream(ground_speed_msg.twist.linear.z,ground_speed_aux);printw(" m/s   ");

    //Pose IMU
    tf2::Matrix3x3 imu_m(tf2::Quaternion (imu_msg.orientation.x,imu_msg.orientation.y,imu_msg.orientation.z,imu_msg.orientation.w));
    r = 0; p = 0; yaw = 0;
    imu_m.getRPY(r, p, yaw);
    if (std::isnan(r)) r = 0.0; 
    if (std::isnan(p)) p = 0.0; 
    if (std::isnan(yaw)) yaw = 0.0; 

    move(7,14);
    printStream(yaw,imu_aux);printw(",");
    move(7,21);
    printStream(p,imu_aux);printw(",");
    move(7,28);
    printStream(r,imu_aux);printw(" rad   ");

    //Speed IMU
    move(8,14);
    printStream(imu_msg.angular_velocity.z,imu_aux);printw(",");
    move(8,21);
    printStream(imu_msg.angular_velocity.y,imu_aux);printw(",");
    move(8,28);
    printStream(imu_msg.angular_velocity.x,imu_aux);printw(" rad/s  ");

    //Acceleration IMU
    move(9,14);
    printStream(imu_msg.linear_acceleration.x,imu_aux);printw(",");
    move(9,21);
    printStream(imu_msg.linear_acceleration.y,imu_aux);printw(",");
    move(9,28);
    printStream(imu_msg.linear_acceleration.z,imu_aux);printw(" m/s2   ");

    //Localization
    //Pose
    move(5,53);
    printStream3(current_pose.pose.position.x,current_pose_aux);printw(",");
    move(5,61);
    printStream3(current_pose.pose.position.y,current_pose_aux);printw(",");
    move(5,69);
    printStream3(current_pose.pose.position.z,current_pose_aux);printw(" m "); 
    //Speed
    move(6,54);
    printStream(current_speed.twist.linear.x,current_speed_aux);printw(",");
    move(6,61);
    printStream(current_speed.twist.linear.y,current_speed_aux);printw(",");
    move(6,68);
    printStream(current_speed.twist.linear.z,current_speed_aux);printw(" m/s "); 
    //Pose(ypr)
    tf2::Matrix3x3 pose_m(tf2::Quaternion (current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w));
    pose_m.getRPY(r, p, yaw);
    if (std::isnan(yaw)) yaw = 0.0; if (std::isnan(r)) r = 0.0; if (std::isnan(p)) p = 0.0;
    move(7,54);
    printStream(yaw,current_pose_aux);printw(",");
    move(7,61);
    printStream(p,current_pose_aux);printw(",");
    move(7,68);
    printStream(r,current_pose_aux);printw(" rad ");     
    //Speed(ypr)
    move(8,54);
    printStream(current_speed.twist.angular.z,current_speed_aux);printw(",");
    move(8,61);
    printStream(current_speed.twist.angular.y,current_speed_aux);printw(",");
    move(8,68);
    printStream(current_speed.twist.angular.x,current_speed_aux);printw(" rad/s ");
    //State
    move(9,54);
    printQuadrotorState();

    //Actuator commands
    if(thrust_aux){
        move(12,19);
        printStream(thrust_msg.pitch,thrust_aux);printw(",");
        move(12,26);
        printStream(thrust_msg.roll,thrust_aux);printw(" rad  ");
        //Speed(z)
        move(13,19);
        printStream(actuator_command_altitude_yaw_msg.twist.linear.z,actuator_command_altitude_yaw_aux);printw(" m/s  ");
        //Thrust
        move(14,19);
        printStream(thrust_msg.thrust.z,thrust_aux);printw(" N  ");
        //Speed(yaw)
        move(15,19);
        printStream(thrust_msg.yaw_rate,thrust_aux);printw(" rad/s  ");
    }else{
        //Pitch roll
        tf2::Matrix3x3 actuator_m(tf2::Quaternion (actuator_command_roll_pitch_msg.pose.orientation.x,actuator_command_roll_pitch_msg.pose.orientation.y,actuator_command_roll_pitch_msg.pose.orientation.z,actuator_command_roll_pitch_msg.pose.orientation.w));
        r = 0; p = 0; yaw = 0;
        actuator_m.getRPY(r, p, yaw);
        if (std::isnan(r)) r = 0.0; 
        if (std::isnan(p)) p = 0.0; 
        move(12,19);
        printStream(p,actuator_command_roll_pitch_aux);printw(",");
        move(12,26);
        printStream(r,actuator_command_roll_pitch_aux);printw(" rad  ");
        //Speed(z)
        move(13,19);
        printStream(actuator_command_altitude_yaw_msg.twist.linear.z,actuator_command_altitude_yaw_aux);printw(" m/s  ");
        //Thrust
        move(14,19);
        printStream(thrust_msg.thrust.z,thrust_aux);printw(" N  ");
        //Speed(yaw)
        move(15,19);
        printStream(actuator_command_altitude_yaw_msg.twist.angular.z,actuator_command_altitude_yaw_aux);printw(" rad/s  ");
    }

    //References
    //Pose
    move(12,53);
    printStream3(current_position_reference.pose.position.x,current_position_reference_aux);printw(",");
    move(12,61);
    printStream3(current_position_reference.pose.position.y,current_position_reference_aux);printw(",");
    move(12,69);
    printStream3(current_position_reference.pose.position.z,current_position_reference_aux);printw(" m "); 
    //Speed
    move(13,54);
    printStream(current_speed_reference.twist.linear.x,current_speed_reference_aux);printw(",");
    move(13,61);
    printStream(current_speed_reference.twist.linear.y,current_speed_reference_aux);printw(",");
    move(13,68);
    printStream(current_speed_reference.twist.linear.z,current_speed_reference_aux);printw(" m/s ");
    //Pose (yaw)
    tf2::Matrix3x3 pose_ref_m(tf2::Quaternion (current_position_reference.pose.orientation.x,current_position_reference.pose.orientation.y,current_position_reference.pose.orientation.z,current_position_reference.pose.orientation.w));
    r = 0; p = 0; yaw = 0;
    pose_ref_m.getRPY(r, p, yaw);
    if (std::isnan(yaw)) yaw = 0.0; 
    move(14,54);
    printStream(yaw,current_position_reference_aux);printw(" rad");
    //Speed (yaw)
    move(15,54);
    printStream(current_speed_reference.twist.angular.z,current_speed_reference_aux);printw(" rad/s  ");  
    //Control mode
    move(16,56);
    printControlMode();  
}

void printNavigationMenu(){
    move(0,0);
    printw("                - ALPHANUMERIC VIEWER OF AERIAL ROBOTICS DATA -");
    move(1,0);
    printw("                        Key: S (sensors), N (navigation)");
    move(2,0);
    printw("                                          ^             ");
    //Measurements
    move(4,0);
    printw(" MEASUREMENTS");
    move(5,0);
    printw(" Pose(z):");
    move(6,0);
    printw(" Speed(xyz):");
    move(7,0);
    printw(" Pose(ypr):");
    move(8,0);
    printw(" Speed(ypr):");
    move(9,0);
    printw(" Accel.(xyz):");

    //Localization
    move(4,42);
    printw("LOCALIZATION");
    move(5,42);
    printw("Pose(xyz):");
    move(6,42);
    printw("Speed(xyz):");
    move(7,42);
    printw("Pose(ypr):");
    move(8,42);
    printw("Speed(ypr):");
    move(9,42);
    printw("Status:");

    //References
    move(11,42);
    printw("REFERENCES");
    move(12,42);
    printw("Pose(xyz):");
    move(13,42);
    printw("Speed(xyz):");
    move(14,42);
    printw("Pose(yaw):");
    move(15,42);
    printw("Speed(yaw):");
    move(16,42);
    printw("Control mode:");

    //Actuator commands
    move(11,0);
    printw(" ACTUATOR COMMANDS");
    move(12,0);
    printw(" Pose(pitch,roll):");
    move(13,0);
    printw(" Speed(z):");
    move(14,0);
    printw(" Thrust:");
    move(15,0);
    printw(" Speed(yaw):");
}

void selfLocalizationSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_speed  = (*msg); 
    current_speed_aux = true;
}
//Callbacks
void selfLocalizationPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose  = (*msg);
    current_pose_aux = true;
}
void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg){ 
    battery_msg=*msg;
    battery_aux = true;
}
void altitudeCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
    altitude_msg=*msg;
    altitude_aux = true;
}
void altitudeSeaLevelCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
    altitude_sea_level_msg=*msg;
    altitude_sea_level_aux = true;
}
void groundSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    ground_speed_msg=*msg;
    ground_speed_aux = true;
}
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg){
    imu_msg=*msg;
    imu_aux = true;
}
void droneStatusCallback(const aerostack_msgs::FlightState::ConstPtr& msg){
    quadrotor_status_msg=*msg;
}
void actuatorCommandRollPitchCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    actuator_command_roll_pitch_msg=*msg;
    actuator_command_roll_pitch_aux = true;
}
void actuatorCommandAltitudeYawCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    actuator_command_altitude_yaw_msg=*msg;
    actuator_command_altitude_yaw_aux = true;
}
void positionRefsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_position_reference = (*msg);
    current_position_reference_aux = true;
}
void speedRefsSubCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    current_speed_reference = (*msg);
    current_speed_reference_aux = true;
}
void controlModeSubCallback(const aerostack_msgs::MotionControlMode::ConstPtr &msg) { 
    if(msg->mode <1 || msg->mode >5 ){
        last_received_control_mode = aerostack_msgs::MotionControlMode::UNKNOWN;
    }else{
        last_received_control_mode = msg->mode;    
    }
}
void temperatureCallback(const sensor_msgs::Temperature::ConstPtr &msg){
    temperature_msg=*msg;
    temperature_aux = true;
}

void thrustSubCallback(const mav_msgs::RollPitchYawrateThrust::ConstPtr &msg){
    thrust_msg = *msg;
    thrust_aux = true;
}
