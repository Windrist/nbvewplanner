#ifndef functions_H
#define functions_H

#include <vector>
#include <numeric>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "ros/ros.h"
#include "mtrand.h"
#include "aslam/PathArray.h"
#include "aslam/TreeArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"


// rdm class, for gentaring random flot numbers
class rdm{
    int i;
    public:
    rdm();
    float randomize();
};

// Norm function prototype
float Norm(std::vector<float>, std::vector<float>);

// Sign function prototype
float Sign(float);

// Nearest function prototype
std::vector<float> Nearest(std::vector<std::vector<float>>, std::vector<float>);

// Steer function prototype
std::vector<float> Steer(std::vector<float>, std::vector<float>, float);

// GridValue function prototype
uint8_t GridValue(nav_msgs::OccupancyGrid &, std::vector<float>);

// ObstacleFree function prototype
uint8_t ObstacleFree(std::vector<float>, std::vector<float> &, nav_msgs::OccupancyGrid);

// GetPrevTree function prototype
void GetPrevTree (aslam::PathArray path, aslam::TreeArray &tree, std::vector<float> point);

// CheckProb function prototype
float CheckProb (nav_msgs::OccupancyGrid &mapData, std::vector<float> point, std::vector<float> robotPose, float eta, int sampleNumber, MTRand &drand);

#endif
