#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>

class TurtleBotController {

public:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher velocity_publisher;
    nav_msgs::Odometry pose;

    ros::Rate rate;
    
    double kp_linear = 0.2;
    double ki_linear = 0.0;
    double kd_linear = 0.1;
    double kp_angular = 1.0;
    double ki_angular = 0.0;
    double kd_angular = 0.5;
    double max_linear_vel = 0.5;
    double max_angular_vel = 1.5;
    double prev_linear_error = 0.0;
    double prev_angular_error = 0.0;
    double integral_linear_error = 0.0;
    double integral_angular_error = 0.0;
    double max_time_to_goal = 60.0;
    double small_movement_threshold = 0.1;
    int max_iterations_without_progress = 50;
    TurtleBotController(): rate(10) {
        odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &TurtleBotController::updatePose, this);
        velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }


    void updatePose(const nav_msgs::Odometry::ConstPtr& data) {
        this->pose = *data;
                // std::cout<<"called"<<std::endl;

    }

    bool moveToGoal(double x_goal, double y_goal) {
        geometry_msgs::Twist vel_msg;
        nav_msgs::Odometry goal_pose;
        bool status =true;
        goal_pose.pose.pose.position.x = x_goal;
        goal_pose.pose.pose.position.y = y_goal;
        double distance_tolerance = 0.1;
        ros::Time start_time = ros::Time::now();
        int iteration = 0;
        double prev_distance = euclideanDistance(goal_pose,this->pose);

        while (euclideanDistance(goal_pose,this->pose) >= distance_tolerance) {
            // Check if max time to reach the goal has elapsed
            // if ((ros::Time::now() - start_time).toSec() > max_time_to_goal) {
            //     ROS_INFO("Max time reached. Unable to reach goal.");
            //     break;
            // }

            // Calculate errors
            // nav_msgs::Odometry curr_pose = ;
            double linear_error = euclideanDistance(goal_pose,this->pose);
            double angular_error = atan2(goal_pose.pose.pose.position.y - this->pose.pose.pose.position.y,
                                          goal_pose.pose.pose.position.x - this->pose.pose.pose.position.x) - getCurrentOrientation();

            // Integral term
            this->integral_linear_error += linear_error;
            this->integral_angular_error += angular_error;

            // PID control for linear velocity
            double linear_vel = kp_linear * linear_error + ki_linear * integral_linear_error + kd_linear * (linear_error - prev_linear_error);
            linear_vel = std::min(std::max(linear_vel, -max_linear_vel), max_linear_vel);

            // PID control for angular velocity
            double angular_vel = kp_angular * angular_error + ki_angular * integral_angular_error + kd_angular * (angular_error - prev_angular_error);
            angular_vel = std::min(std::max(angular_vel, -max_angular_vel), max_angular_vel);

            // Update previous errors
            prev_linear_error = linear_error;
            prev_angular_error = angular_error;

            // Check if movement is small
            if (linear_error < small_movement_threshold) {
                ROS_INFO("Reached close to the goal. Stopping.");
                status = true;
                stopRobot();
                return status;
                // break; 
            }

            // Publish the velocity message
            vel_msg.linear.x = linear_vel;
            vel_msg.angular.z = angular_vel;
            velocity_publisher.publish(vel_msg);
            std::cout<<"pose :: "<<this->pose.pose.pose.position.x<<std::endl;
            std::cout<<"iteration :: "<<iteration<<std::endl;
            std::cout<<"euclideanDistance :: "<<euclideanDistance(goal_pose,this->pose)<<std::endl;
            std::cout<<"linear velo :: "<<linear_vel<<std::endl;
            std::cout<<"angular velo :: "<<angular_vel<<std::endl;
            // Sleep for the specified rate
            rate.sleep();

            // Check for progress
            if (iteration > max_iterations_without_progress) {
                double current_distance = euclideanDistance(goal_pose,this->pose);
                if (current_distance >= prev_distance) {
                    ROS_INFO("Not making progress. Stopping.");
                    // return false;//chang
                            stopRobot();
                            status = false;
                            return status;
                    break;
                }
                prev_distance = current_distance;
                iteration = 0;
            }
            iteration++;

            ros::spinOnce();//This statement is very important so as to update the data continuously.
        }

        // Stop the robot when the goal is reached
        stopRobot();

        return status;

    }

    double euclideanDistance(const nav_msgs::Odometry& goal_pose,const nav_msgs::Odometry& curr_pose) {
        return std::sqrt(std::pow(goal_pose.pose.pose.position.x - curr_pose.pose.pose.position.x, 2) +//changes made
                         std::pow(goal_pose.pose.pose.position.y - curr_pose.pose.pose.position.y, 2));//changes made
    }

    double getCurrentOrientation() {
        return std::atan2(2 * (this->pose.pose.pose.orientation.w * this->pose.pose.pose.orientation.z + this->pose.pose.pose.orientation.x * this->pose.pose.pose.orientation.y),
                          1 - 2 * (std::pow(this->pose.pose.pose.orientation.y, 2) + std::pow(this->pose.pose.pose.orientation.z, 2)));
    }

    void stopRobot() {
        geometry_msgs::Twist vel_msg;
        velocity_publisher.publish(vel_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot_controller_cpp");
            TurtleBotController controller;

    double x_goal = -1, y_goal=2;
    // while(ros::ok()){
    //     ros::spinOnce();
    //     // ROS_INFO_STREAM("Moving to goal: (" << x_goal << ", " << y_goal << ")");
    //     // controller.moveToGoal(x_goal, y_goal);
    //     bool status = controller.moveToGoal(x_goal, y_goal);
    //     if(status){
    //         break;
    //     }
    //     std::cout<<"loop running"<<status<<std::endl;
    // }
    ros::spinOnce();
    bool status = controller.moveToGoal(x_goal, y_goal);
    std::cout<<status<<std::endl;
    if(status){
        std::cout<<"Successfully Moved to the required coordinates"<<std::endl;

    }else{
        std::cout<<"Failed to move to the required coordinates!!"<<std::endl;

    }
    controller.moveToGoal(x_goal, y_goal);


    

    return 0;
}