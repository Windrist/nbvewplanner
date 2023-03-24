#include "functions.h"


// rdm class, for generating random float numbers
rdm::rdm() {
    i = time(0);
}


float rdm::randomize() {
    i = i + 1;
    srand(i);
    return float(rand()) / float(RAND_MAX);
}


// Norm function 
float Norm(std::vector<float> x1, std::vector<float> x2) {
    return pow(pow(x2[0] - x1[0], 2) + pow(x2[1] - x1[1], 2), 0.5);
}


// Sign function
float Sign(float n) {
    if (n < 0.0)
        return -1.0;
    else
        return 1.0;
}


// Nearest function
std::vector<float> Nearest(std::vector<std::vector<float>> V, std::vector<float> x) {
    auto min = Norm(V[0], x); 
    int min_index;
    float temp;

    for (int j=0; j<V.size(); j++) {
        temp = Norm(V[j], x);
        if (temp <= min) {
            min = temp;
            min_index = j;
        }
    }
    return V[min_index];
}


// Steer Function
std::vector<float> Steer(std::vector<float> x_nearest, std::vector<float> x_rand, float eta) {
    std::vector<float> x_new;

    if (Norm(x_nearest, x_rand) <= eta)
        x_new = x_rand;
    else {
        auto m = (x_rand[1] - x_nearest[1]) / (x_rand[0] - x_nearest[0]);

        x_new.push_back(Sign(x_rand[0] - x_nearest[0]) * sqrt(pow(eta, 2) / (pow(m, 2) + 1)) + x_nearest[0]);
        x_new.push_back(m * (x_new[0] - x_nearest[0]) + x_nearest[1]);

        if (x_rand[0] == x_nearest[0]) {
            x_new[0] = x_nearest[0];
            x_new[1] = x_nearest[1] + eta;
        }
    }
    return x_new;
}


// GridValue Function
uint8_t GridValue(nav_msgs::OccupancyGrid &mapData, std::vector<float> Xp) {

    auto resolution = mapData.info.resolution;
    auto Xstartx = mapData.info.origin.position.x;
    auto Xstarty = mapData.info.origin.position.y;

    auto width = mapData.info.width;
    auto Data = mapData.data;

    // Returns Grid Value at "Xp" Location
        // Map data: 100 == Occupied
        // -1 == Unknown
        // 0 == Free
        // 1 -> 99 == Observation Probability
    // 
    auto indx = floor((Xp[1] - Xstarty) / resolution) * width + floor((Xp[0] - Xstartx) / resolution);
    auto out = Data[int(indx)];
    return out;
}


// ObstacleFree Function
uint8_t ObstacleFree(std::vector<float> xnear, std::vector<float> &xnew, nav_msgs::OccupancyGrid mapsub) {
    auto rez = mapsub.info.resolution;
    auto stepz = int(ceil(Norm(xnew, xnear)) / rez); 
    auto xi = xnear;
    bool obs = false, unk = false;
    
    geometry_msgs::Point p;
    for (int c=0; c<stepz; c++) {
        xi = Steer(xi, xnew, rez);

        if (GridValue(mapsub, xi) == 100) {
            obs = true;
            break;
        }
        
        if (GridValue(mapsub, xi) == -1) {
            unk = true;
            break;
        }
    }
    uint8_t out;
    xnew = xi;

    if (unk)
        out = -1;
    if (obs)
        out = 0;
    if (!unk && !obs)
        out = 1;
        
    return out;
}


// GetPrevTree Function
void GetPrevTree (aslam::PathArray path, aslam::TreeArray &tree, std::vector<float> point) {
    std::for_each(path.trees.begin(), path.trees.end(), [&] (auto x) {
        if (x.node.point.x == point[0] && x.node.point.y == point[1])
            tree = x;
    });
}


// CheckProb Function
float CheckProb (nav_msgs::OccupancyGrid &mapData, std::vector<float> point, std::vector<float> robotPose, float eta, int sampleNumber, MTRand &drand) {
    auto resolution = mapData.info.resolution;
    auto Xstartx = mapData.info.origin.position.x;
    auto Xstarty = mapData.info.origin.position.y;

    auto width = mapData.info.width;
    auto Data = mapData.data;
    std::vector<float> sp;

    // Returns Grid Value at "Xp" Location
        // Map data: 100 == Occupied
        // -1 == Unknown
        // 0 == Free
        // 1 -> 49 == Free Probability
        // 51 -> 99 == Obstacle Probability
    //
    uint8_t sigma = 50;
    float probAll = 0, dist = Norm(point, robotPose);

    for (int i=0; i<sampleNumber; i++) {
        sp.push_back((drand() - 0.5) * eta + point[0]);
		sp.push_back((drand() - 0.5) * eta + point[1]);
        auto value = GridValue(mapData, sp);
        if (value > sigma) {
            auto probGain = exp(-0.5 * (((value - sigma) / sigma) ^ 2)) / sqrt(2 * M_PI * (sigma ^ 2));
            probAll += probGain;
        }
        else if (value == 1) {
            auto probGain = 1 / sqrt(2 * M_PI);
            ROS_INFO("Prob: %f", probGain);
            probAll += probGain;
        }
        sp.clear();
    }

    probAll = probAll / sampleNumber;

    return probAll;
}
 


     
   


  
 
 
 
 





























