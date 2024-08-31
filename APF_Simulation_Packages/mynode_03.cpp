#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <vector>
// #include "matplotlibcpp.h"


// namespace plt = matplotlibcpp;
// Define constants
const double zeta = 1.0; 
const double thresholdA = 1.0;
const double eta = 1.0;  // Repulsive force gain
const double Q_star = 2.0; // Threshold distance for obstacles

// Define point structure
struct Point {
    double x;
    double y;
};


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

Point moveRobot(Point start,std::vector<Point> obstacles,Point goal){
    Point resultantForce = calculateResultantForce(start,obstacles,goal);
    Point newPos;
    newPos.x = start.x + resultantForce.x;
    newPos.y = start.y + resultantForce.y;
    return newPos;
}

void callback(const sensor_msgs::LaserScanConstPtr& msg, std::vector<Point>& obstacles){
    std::cout<<"called"<<std::endl;
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

int main(int argc,char** argv){
    ros::init(argc,argv,"cluster");
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
        // if(obstacles.size()!=0){
        //     break;
        // }
    }

    return 0;
}