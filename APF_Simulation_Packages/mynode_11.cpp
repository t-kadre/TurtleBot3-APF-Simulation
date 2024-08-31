#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
// #include "matplotlibcpp.h"


// namespace plt = matplotlibcpp;
// Define constants
const double zeta = 1.0; 
const double thresholdA = 1.0;
const double eta = 1.0;  // Repulsive force gain
const double Q_star = 3.0; // Threshold distance for obstacles
const double alpha = 0.5;
// Define point structure
struct Point {
    double x;
    double y;
};

double normalize_angle(double angle){
    if (fmod(angle, 2*M_PI) > M_PI) {
        return fmod(angle, M_PI) - M_PI;
    }else{
        return fmod(angle, M_PI);
    }
}


Point vectorRotation(Point v,double theta){
    return {v.x*cos(theta)+(-1)*v.y*sin(theta),v.x*sin(theta)+v.y*cos(theta)};
}

Point rntTransformation(Point p,Point translationPoint){
    Point newPoint;
    double rotationAngle = atan(translationPoint.y/translationPoint.x);

    newPoint.x = (p.x-translationPoint.x)*cos(rotationAngle) + (p.y-translationPoint.y)*sin(rotationAngle);
    newPoint.y = (-1)*(p.x-translationPoint.x)*sin(rotationAngle) + (p.y-translationPoint.y)*cos(rotationAngle);
    return newPoint;
}

Point calculateAttractiveForce(Point robopos,Point goal){
    double dx = goal.x - robopos.x;
    double dy = goal.y - robopos.y;
    double distance = sqrt(dx*dx + dy*dy);
    if (distance > thresholdA) {
        return {(zeta * dx) / distance, (zeta * dy) / distance};
    } else {
        return {0, 0};
    }

}
Point calculateRepulsiveForce(Point robopos,std::vector<Point> obstacles) {
    // Point robotPos = robot.getPosition();
    double repulsiveForceX = 0, repulsiveForceY = 0;
    
    for (const auto& obstacle : obstacles) {
        double dx = robopos.x - obstacle.x;
        double dy = robopos.y - obstacle.y;
        double distance = sqrt(dx*dx + dy*dy);      
        
        if (distance <= Q_star) {
            repulsiveForceX += (eta * (1/distance - 1/Q_star) * (1/distance*distance*distance) * dx);
            repulsiveForceY += (eta * (1/distance - 1/Q_star) * (1/distance*distance*distance) * dy);
        }
    }
    
    return {repulsiveForceX, repulsiveForceY};
}

Point calculateResultantForce(Point robopos,std::vector<Point> obstacles,Point goal) {
    Point attractiveForce = calculateAttractiveForce(robopos,goal);
    Point repulsiveForce = calculateRepulsiveForce(robopos,obstacles);

    return {attractiveForce.x + repulsiveForce.x, attractiveForce.y + repulsiveForce.y};
}

std::pair<Point,Point> moveRobot(Point start,std::vector<Point> obstacles,Point goal){
    Point resultantForce = calculateResultantForce(start,obstacles,goal);
    Point newPos;
    newPos.x = start.x + alpha*resultantForce.x;
    newPos.y = start.y + alpha*resultantForce.y;
    return {newPos,resultantForce};
}

void callback(const sensor_msgs::LaserScanConstPtr& msg, std::vector<Point>& obstacles){
    // std::cout<<"called"<<std::endl;
    bool flag=false;
    int count=0;
    double dmin = INT_MAX;
    int ind = -1;
    obstacles.clear();

    for(int i=0;i<msg->ranges.size();i++){
        if(msg->ranges[i]!= std::numeric_limits<float>::infinity())
        {   
            flag = true;
            if(dmin>double(msg->ranges[i])){
                dmin  = double(msg->ranges[i]);
                ind = i;
            }
        }
        if((msg->ranges[i]== std::numeric_limits<float>::infinity() && flag) || (i==msg->ranges.size() && flag)){
            count++;
            flag=false;
            obstacles.push_back({dmin*cos(ind*M_PI/180),dmin*sin(ind*M_PI/180)});
            dmin =INT_MAX;
            ind =-1;
        }

    }



}

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
            // std::cout<<"pose :: "<<this->pose.pose.pose.position.x<<std::endl;
            // std::cout<<"iteration :: "<<iteration<<std::endl;
            // std::cout<<"euclideanDistance :: "<<euclideanDistance(goal_pose,this->pose)<<std::endl;
            // std::cout<<"linear velo :: "<<linear_vel<<std::endl;
            // std::cout<<"angular velo :: "<<angular_vel<<std::endl;
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



class TurtleBotRotationController {

public:
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    ros::Publisher velocity_publisher;
    nav_msgs::Odometry pose;

    ros::Rate rate;
    
    double kp_angular = 1.0;
    double ki_angular = 0.0;
    double kd_angular = 0.5;
    double max_angular_vel = 1.5;
    double prev_angular_error = 0.0;
    double integral_angular_error = 0.0;
    double max_time_to_goal = 60.0;
    double small_movement_threshold = 0.05;
    int max_iterations_without_progress = 50;
    TurtleBotRotationController(): rate(10) {
        odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &TurtleBotRotationController::updatePose, this);
        velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    }


    void updatePose(const nav_msgs::Odometry::ConstPtr& data) {
        this->pose = *data;

    }

    bool moveToGoal(double theta_goal) {
        geometry_msgs::Twist vel_msg;
        bool status =true;

        double angle_tolerance = 0.05;
        ros::Time start_time = ros::Time::now();
        int iteration = 0;
            std::cout<<"current :: "<<this->pose.pose.pose.orientation.z<<std::endl;

        while (std::abs(this->getCurrentOrientation() - theta_goal) >= angle_tolerance) {

            std::cout<<"hello"<<std::endl;

            double angular_error = theta_goal- getCurrentOrientation();

            this->integral_angular_error += angular_error;

            // PID control for angular velocity
            double angular_vel = kp_angular * angular_error + ki_angular * integral_angular_error + kd_angular * (angular_error - prev_angular_error);
            angular_vel = std::min(std::max(angular_vel, -max_angular_vel), max_angular_vel);

            // Update previous errors

            prev_angular_error = angular_error;

            // Check if movement is small
            if (abs(angular_error) < small_movement_threshold) {
                ROS_INFO("Reached close to the goal. Stopping.");
                status = true;
                stopRobot();
                return status;
                // break; 
            }

            // Publish the velocity message

            vel_msg.angular.z = angular_vel;
            velocity_publisher.publish(vel_msg);
            std::cout<<"pose :: "<<this->pose.pose.pose.orientation.z<<std::endl;
            std::cout<<"iteration :: "<<iteration<<std::endl;
            std::cout<<"angular velo :: "<<angular_vel<<std::endl;
            // Sleep for the specified rate
            rate.sleep();

            // Check for progress
            if (iteration > max_iterations_without_progress) {

                    ROS_INFO("Not making progress. Stopping.");
                    // return false;//chang
                            status = false;

                            stopRobot();
                            return status;
                    break;

            }
            iteration++;

            ros::spinOnce();//This statement is very important so as to update the data continuously.
        }

        // Stop the robot when the goal is reached
        stopRobot();

        return status;

    }

    double getCurrentOrientation() {
        return std::atan2(2 * (this->pose.pose.pose.orientation.w * this->pose.pose.pose.orientation.z + this->pose.pose.pose.orientation.x * this->pose.pose.pose.orientation.y),
                          1 - 2 * (std::pow(this->pose.pose.pose.orientation.y, 2) + std::pow(this->pose.pose.pose.orientation.z, 2)));
    }

    void stopRobot() {
        geometry_msgs::Twist vel_msg;
        vel_msg.angular.z=0;
        velocity_publisher.publish(vel_msg);
    }
};



int main(int argc,char** argv){
    ros::init(argc,argv,"cluster");
    TurtleBotController controller;
    TurtleBotRotationController rotationController;
    ros::NodeHandle node_handle; 
    std::vector<Point> obstacles;   
    ros::Subscriber subscriber = node_handle.subscribe<sensor_msgs::LaserScan>("/scan",20, boost::bind(callback, _1, boost::ref(obstacles)));
        // ros::spin();

    while(ros::ok()){
        ros::spinOnce();
        std::cout<<obstacles.size()<<'\n';          
        for(int i=0;i<obstacles.size();i++){
        if(i==0){
            std::cout<<"[";
        }
        std::cout<<"{"<<obstacles[i].x<<","<<obstacles[i].y<<"}"<<",";
        if(i==obstacles.size()-1){
            std::cout<<"]";
        }
        }
        if(obstacles.size()!=0){
            break;
        }
    }




    Point start = {0,0};
    Point goal = {10,10};
    Point plotPoint;
    // vector<double> x,y;
    // std::vector<double> obs_x, obs_y;
    // for (const auto& obstacle : obstacles) {
    //     Point obstaclePos = obstacle;
    //     obs_x.push_back(obstaclePos.x);
    //     obs_y.push_back(obstaclePos.y);
    // }
    plotPoint = start;
    // x.push_back(start.x);
    // y.push_back(start.y);  

    Point resultantForce;
    double prevAngle = 0;
    while(1) { // Simulate 100 iterations

        resultantForce = moveRobot(start,obstacles,goal).second;
        
        start = moveRobot(start,obstacles,goal).first;

        prevAngle+=atan(resultantForce.y/resultantForce.x);
        resultantForce = vectorRotation(resultantForce,prevAngle);
        plotPoint = {plotPoint.x+resultantForce.x,plotPoint.y+resultantForce.y};
        
        ros::spinOnce();

        bool status = controller.moveToGoal(plotPoint.x, plotPoint.y);
        std::cout<<status<<std::endl;

        if(status){
            std::cout<<"Successfully Moved to the required coordinates"<<std::endl;

        }else{
            std::cout<<"Failed to move to the required coordinates!!"<<std::endl;
            bool status = controller.moveToGoal(plotPoint.x, plotPoint.y);
        }
                // ros::spinOnce();
        double turnAngle = normalize_angle(prevAngle);
        bool status2 = rotationController.moveToGoal(turnAngle);
        std::cout<<status2<<std::endl;

        if(status2){
            std::cout<<"Successfully Moved to the required coordinates"<<std::endl;

        }else{
            std::cout<<"Failed to move to the required coordinates!!"<<std::endl;
            bool status2 = rotationController.moveToGoal(turnAngle);
        }

        // bool status1 = false;
        // std::cout<<status<<std::endl;

        // while(!status1){
        //     status1 = controller.moveToGoal(plotPoint.x, plotPoint.y);
        // }
        // if(status1){
        //     std::cout<<"Successfully Moved to the required coordinates"<<std::endl;
        // }else{
        //     std::cout<<"Failed to move to the required coordinates!!"<<std::endl;
        //     // bool status = controller.moveToGoal(plotPoint.x, plotPoint.y);
        // }

        goal = {rntTransformation(goal,{start.x,start.y}).x,rntTransformation(goal,{start.x,start.y}).y};
        for(auto &obstacle : obstacles){
            obstacle = {rntTransformation(obstacle,start).x,rntTransformation(obstacle,start).y};
            std::cout<<"obstacle :: "<<obstacle.x<<" | "<<obstacle.y<<std::endl;
        }
        // x.push_back(plotPoint.x);
        // y.push_back(plotPoint.y);

        start = {0,0};
        std::cout<<"plotPoint :: "<<plotPoint.x<<" |---| "<<plotPoint.y<<std::endl;

        std::cout<<"goal :: "<<goal.x<<" |---| "<<goal.y<<std::endl;
        double d = sqrt((start.x-goal.x)*(start.x-goal.x) + (start.y-goal.y)*(start.y-goal.y));

        if(d<0.5){
            break;
        }

    }
    //     // Plot robot path
    // plt::plot(x, y, {{"color", "blue"}, {"label", "Robot Path"}});
    
    // plt::scatter(obs_x, obs_y, 100, {{"color", "red"}, {"marker", "o"}, {"label", "Obstacles"}});

    // // Add labels and legend
    // plt::xlabel("X");
    // plt::ylabel("Y");
    // plt::title("Path Planning with Artificial Potential Field");
    // plt::legend();

    // // Show the plot
    // plt::show();
}