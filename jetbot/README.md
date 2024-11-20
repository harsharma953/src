# mapping steps
--> launch the rviz in dev machine and add the different components 
    --> rviz2
    --> add tf , robotmodel , map --for local, map--> for costmap , laserscan 

--> launch joint_state_publisher gui node in dev machine --> ros2 run joint_state_publisher_gui joint_state_publisher_gui


--> launch rplidar and slam_toolbox node --> ros2 launch jetbot main.launch.py 

--> launch the robot_state_publisher --> ros2 launch jetbot robot_rviz_launch.py

--> launch the odom node for calculating the odom of vehicle --> python3 odometry_publisher.py 

--> launch the keyboard listener for giving the commands to bot and receiving the encoder value --> python3 publish_encoders.py

