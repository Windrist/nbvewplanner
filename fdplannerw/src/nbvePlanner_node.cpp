#include <ros/ros.h>
#include "fdplannerw/nbve.hpp"
#include "fdplannerw/rrt.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "nbvep");
    ros::NodeHandle nh;

    nbvePlanner::Planner planner(nh);

    ros::spin();
    return 0;
}
