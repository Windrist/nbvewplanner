#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "fdplannerw/nbvep_srv.h"
#include "fdplannerw/nbvep_resrv.h"
#include "fdplannerw/CountVoxel.h"


// Global variables
visualization_msgs::Marker points, line;
ros::Time startTime, cutoffTime;
ros::Publisher mapCountPub;
int32_t offTime = 300;

void mapCallback(const nav_msgs::OccupancyGrid& msg) {
    fdplannerw::CountVoxel countVox;
    countVox.header.stamp = ros::Time::now();
    countVox.header.frame_id = "map";
    countVox.info = msg.info;
    for (uintptr_t count=0; count < msg.data.size(); count++) {
        if (msg.data.at(count) == 0) countVox.freeVox.data += 1;
        if (msg.data.at(count) == -1) countVox.uknVox.data += 1;
        else countVox.occVox.data += 1;
    }
    mapCountPub.publish(countVox);
    cutoffTime = ros::Time::now();
    
    if (ros::Time::now() - startTime >= ros::Duration(offTime)) {
        ros::shutdown();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "exploration");
    ros::NodeHandle nh;

    ros::Subscriber mapSub = nh.subscribe("/map", 1000, &mapCallback);
    mapCountPub = nh.advertise<fdplannerw::CountVoxel>("/countArea", 1000);

    std::string ns = ros::this_node::getName();
    if (!nh.getParam(ns + "/maxTime", offTime)) {
        ROS_WARN("No cutoff Time Defined. Looking for %s. Default is 300.", (ns + "/maxTime").c_str());
    }

    ros::Rate rate(10);

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action("/move_base", true);
    action.waitForServer();

    move_base_msgs::MoveBaseGoal goal;
    fdplannerw::nbvep_srv planSrv;
    fdplannerw::nbvep_resrv planReSrv;
    
    goal.target_pose.header.frame_id = "map";
    planSrv.request.header.frame_id = "map";
    bool stateFlag = true;

    // Start planning: The planner is called and the computed path sent to the controller.
    uint32_t iteration = 0;
    startTime = ros::Time::now();
    cutoffTime = ros::Time::now();

    while (ros::ok()) {
        if (stateFlag) {
            ROS_INFO("Start Planner!");
            if (ros::service::call("nbveplanner", planSrv) && planSrv.response.path.size() != 0) {
                planReSrv.request.path = planSrv.response.path;
                planSrv.response.path.clear();
                planReSrv.request.init.data = true;

                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose = planReSrv.request.path.back();
                planReSrv.request.path.pop_back();
                action.sendGoal(goal);

                if (ros::service::call("nbvereplanner", planReSrv)) {
                    planReSrv.request.init.data = false;
                    stateFlag = false;

                    ROS_INFO("Planning iteration %i", iteration);
                    iteration++;
                }
            }
            else {
                ROS_WARN("Planner not reachable");
                continue;
            }
        }
        
        if (planReSrv.response.path.size() != 0 && !stateFlag) {
            if (action.getState() == actionlib::SimpleClientGoalState::ACTIVE) {
                planReSrv.request.path = planReSrv.response.path;
                if (ros::service::call("nbvereplanner", planReSrv)) {
                    ROS_DEBUG("Start Replanning State!");
                }
                else {
                    ROS_ERROR("Cannot Replanning!");
                }
            }
            if (action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose = planReSrv.response.path.back();
                planReSrv.response.path.pop_back();
                action.sendGoal(goal);
            }
            action.waitForResult(ros::Duration(rate));
        }
        else if (action.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            stateFlag = true;
        }

        ros::spinOnce();
		rate.sleep();
    }
}
