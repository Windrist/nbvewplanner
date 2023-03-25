#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>


nav_msgs::Path path, deadPath;

ros::Publisher pathPub, deadpathPub;

void odomCallback(const nav_msgs::Odometry& msg) {
    path.header = msg.header;
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    path.poses.push_back(pose);
    pathPub.publish(path);
}

void deadCallback(const nav_msgs::Odometry& msg) {
    deadPath.header = msg.header;
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose = msg.pose.pose;
    deadPath.poses.push_back(pose);
    // deadpathPub.publish(deadPath);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "visualizePath");

    ros::NodeHandle nh;
    ros::Subscriber odomSub = nh.subscribe("/odom", 1000, &odomCallback);
    // ros::Subscriber deadSub = nh.subscribe("/dead_reackoning", 1000, &deadCallback);
    pathPub = nh.advertise<nav_msgs::Path>("/path", 1000);
    deadpathPub = nh.advertise<nav_msgs::Path>("/dead_path", 1000);

    ros::spin();
}
