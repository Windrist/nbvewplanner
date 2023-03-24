#ifndef RHE_PLANNER_HPP_  // NOLINT
#define RHE_PLANNER_HPP_

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include "nbvplannerw/rrt.hpp"
#include "nbvplannerw/pe.hpp"
#include "nbvplannerw/nbvep_srv.h"
#include "nbvplannerw/nbvep_resrv.h"


namespace nbvePlanner {

    class Planner {
        private:
            ros::NodeHandle nh_;
            ros::ServiceServer plannerService_;
            ros::ServiceServer rePlannerService_;
            ros::Subscriber posClient_;
            ros::Subscriber odomClient_;
            ros::Subscriber mapClient_;
            ros::Publisher weightMapPub_;
            ros::Publisher robPosPub_;
            ros::Publisher pathPub_;
            ros::Publisher comTPub_;

            tf::TransformListener listener_;
            tf::StampedTransform transform_;

            nav_msgs::OccupancyGrid map_;

            RRT* tree_;
            PE* rePlanner_;
            CollisionDetector cd_;

            RRT::Params params_;
            PE::Params paramsRePlanner_;
            
            double resolution_;

            bool initialized_{false};
            bool mapReady_{false};
            

        public:
            Planner(const ros::NodeHandle& nh);

            bool setParams();

            void posCallback(const geometry_msgs::PoseWithCovarianceStamped& pose);
            void odomCallback(const nav_msgs::Odometry& pose);
            void mapCallback(const nav_msgs::OccupancyGrid& msg);

            bool plannerCallback(nbvplannerw::nbvep_srv::Request& req, nbvplannerw::nbvep_srv::Response& res);
            bool rePlannerCallback(nbvplannerw::nbvep_resrv::Request& req, nbvplannerw::nbvep_resrv::Response& res);

    };

}  // namespace nbvePlanner

#endif  // RHE_PLANNER_HPP_  // NOLINT
