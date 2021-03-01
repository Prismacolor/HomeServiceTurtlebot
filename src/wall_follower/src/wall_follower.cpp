#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"

// these commands set up pub, sub, and msgs for motor commands
ros::Publisher motor_command_publisher;
ros::Subscriber laser_subscriber;
sensor_msgs::LaserScan laser_msg;
geometry_msgs::Twist motor_command;

// this will help us define the robot's movements
typedef enum ROBOT_MOVEMENT{
    STOP = 0,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    MOVE_LEFT,
    MOVE_RIGHT
} ROBOT_MOVEMENT;

// establish what the robot's move was and update motor_command
bool moving_robot(const ROBOT_MOVEMENT robot_move){
    if (robot_move == STOP){
        ROS_INFO("Robot has stopped.");
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.0;
    }
    else if (robot_move == FORWARD){
        ROS_INFO("Robot has moved forward.");
        motor_command.angular.z = 0.0;
        motor_command.linear.x = 0.5;
    }
    else if (robot_move == BACKWARD){
        ROS_INFO("Robot has moved backward.");
        motor_command.angular.z = 0.0;
        motor_command.linear.x = -0.5;
    }
    else if (robot_move == TURN_LEFT){
        ROS_INFO("Robot has turned to the left.");
        motor_command.angular.z = 0.5;
        motor_command.linear.x = 0.0;
    }
    else if (robot_move == TURN_RIGHT){
        ROS_INFO("Robot has turned to the right.");
        motor_command.angular.z = -0.5;
        motor_command.linear.x = 0.0;
    }
    else if (robot_move == MOVE_LEFT){
        ROS_INFO("Robot has moved to the left.");
        motor_command.angular.z = 0.5;
        motor_command.linear.x = 0.5;
    }
    else if (robot_move == MOVE_RIGHT){
        ROS_INFO("Robot has moved to the right.");
        motor_command.angular.z = 0.5;
        motor_command.linear.x = 0.5;
    } else {
        ROS_INFO("Invalid movement type.");
        return false;
    }

    motor_command_publisher.publish(motor_command);
    ros::Duration(.01).sleep();
    return true;
}

// additional variables for the robot's state
bool following_wall = false;
bool doorway = false;
bool hit_obstacle = false;

//callback function we run after each laser scan
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
    // take in the laser info and process it
    laser_msg = *scan_msg;
    std::vector<float> laser_ranges;
    laser_ranges = laser_msg.ranges;
    size_t range_size = laser_ranges.size();

    float left_ = 0.0;
    float right_ = 0.0;
    float new_range_min = laser_msg.range_max;
    float new_range_max = laser_msg.range_min;
    int nan_count = 0;

    for(size_t i = 0; i < range_size; i++) {
        if(isnan(laser_ranges[i])) {
            nan_count++;
        }
        if(laser_ranges[i] < new_range_min){
            new_range_min = laser_ranges[i];
        }
        if(i < range_size / 4){
            if(laser_ranges[i] > new_range_max) {
                new_range_max = laser_ranges[i];
            }
        }
        if (i > range_size / 2) {
            left_ += laser_ranges[i];
        } else {
            right_ += laser_ranges[i];
        }
    }

    // check to see if the robot has crashed
    if (nan_count > (range_size * 0.9) || laser_ranges[range_size / 2] < 0.25) {
        hit_obstacle = true;
    }

    if (!hit_obstacle){
        // hooray the robot did not crash!
        if(new_range_min <= 0.5 && !doorway){
            following_wall = true;
            moving_robot(STOP);
          
            if(left_ >= right_){
                moving_robot(TURN_RIGHT);
            } else {
                moving_robot(TURN_LEFT);
            } 
        } else {
            ROS_INFO("We may have encountered a door.", new_range_max, following_wall);
            moving_robot(STOP);
            if(following_wall){
                if(new_range_max >= 2.0){
                    doorway = true;
                    following_wall = false;
                    ROS_INFO("New Range Max > 2.0", new_range_max);
                }
            }
            if(doorway){
                 if(laser_ranges[0] <= 0.5){
                     doorway = false;
                 } else {
                    moving_robot(MOVE_RIGHT);
                 }
                 ROS_INFO("Robot has hit a door. Turning right.");
            } else {
                moving_robot(FORWARD);
            }
       }
    } else {
          // the robot crashed
          moving_robot(BACKWARD);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "wall_follower");
    ros::NodeHandle n;
   
    // set up publisher and subcribers for motor command and lasers 
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100);
    laser_subscriber = n.subscribe("/scan", 1000, laser_callback);

    ros::Duration time_between_ros_actions(0.001);
    while(ros::ok()){
        ros::spinOnce();
        time_between_ros_actions.sleep();
    }

    return 0;
}

