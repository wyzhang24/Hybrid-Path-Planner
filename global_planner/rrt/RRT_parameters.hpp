#ifndef DEFINE_RRT_PARA_H
#define DEFINE_RRT_PARA_H

#include <vector>
#include <math.h>
#include "../def_all.hpp"


// const double PI = 3.14159;
// const double D2R = PI/180.0;
// const double R2D = 180.0/PI;

const double step_len = 20/40.0;

const double xrange_map = 2000/100.0;
const double yrange_map = 2000/100.0;

const double max_yaw_range = 30.0 * PI / 180.0; // the maximum yaw it can reach
const int max_attemp_num = 10; //cf: 3
extern std::vector<std::vector<double>> obstacles = {};


const double car_radius = 10 /20.0;

// using haokun's parameters
extern std::vector<double> goal = {};
extern std::vector<double> start = {};

const double threshod = 50 / 80.0;

const int max_iteration = 500000;

const std::string trapped = "Trapped";
const std::string reached = "Reached";
const std::string advanced = "Advanced";



#endif