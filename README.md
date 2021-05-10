# Alphanumeric Viewer

![Alphanumeric Viewer](https://i.ibb.co/rwbJBj3/alpha1.png)
![Alphanumeric Viewer](https://i.ibb.co/MSvyQm7/alpha2.png)

# Subscribed topics

- **self_localization/flight_state** ([aerostack_msgs/flightState](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/7c07e4317e20a1142226d513336a06a2ff585629/msg/FlightState.msg))   
Flight state (landed, hovering, ...)

- **actuator_command/roll_pitch** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))           
Actuator command for the multirotor specifying roll and pitch (the rest of values for <x, y, z, yaw> are discarded).

- **actuator_command/roll_pitch_yaw_rate_thrust** ([mav_msgs/RollPitchYawrateThrust](http://docs.ros.org/api/mav_msgs/html/msg/RollPitchYawrateThrust.html))           
Actuator command for the multirotor specifying roll (rad), pitch (rad), yaw rate (rad/s) and thrust (N: Newtons).

- **actuator_command/altitude_rate_yaw_rate** ([geometry_msgs/TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))           
Actuator command for the multirotor specifying rates for altitude (d_z) and yaw (d_yaw) (the rest of values for <d_x, d_y, d_yaw, d_pitch, d_roll> are discarded).

- **motion_reference/assumed_control_mode** ([aerostack_msgs/MotionControlMode](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/msg/MotionControlMode.msg))  
Current controller's control mode.

- **motion_reference/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))  
Pose reference for the controller.

- **motion_reference/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))  
Speed reference for the controller.

- **self_localization/pose** ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))      
Current pose of the vehicle

- **self_localization/speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))      
Current speed of the vehicle

- **sensor_measurement/battery_state** ([sensor_msgs/BatteryState](http://docs.ros.org/api/sensor_msgs/html/msg/BatteryState.html))   
Battery state.

- **sensor_measurement/altitude** ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))   
Altitude respect to the ground.

- **sensor_measurement/sea_level_altitude** ([geometry_msgs/PointStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PointStamped.html))   
Sea level altitude.

- **sensor_measurement/temperature** ([sensor_msgs/Temperature](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Temperature.html))   
Temperature reading.

- **sensor_measurement/imu** ([sensor_msgs/Imu](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))   
Inertial Measurement Unit data.

- **sensor_measurement/linear_speed** ([geometry_msgs/TwistStamped](http://docs.ros.org/lunar/api/geometry_msgs/html/msg/TwistStamped.html))   
Direct speed estimated by odometry sensors.


---
# Contributors
**Code Maintainer:** Alberto Rodelgo  
**Author:** Alberto Rodelgo