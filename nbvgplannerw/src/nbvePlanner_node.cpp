#include <ros/ros.h>
#include "nbvgplannerw/nbve.hpp"
#include "nbvgplannerw/rrt.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "nbvep");
    ros::NodeHandle nh;

    nbvePlanner::Planner planner(nh);

    ros::spin();
    return 0;
}
