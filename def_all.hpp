// Include all global viarables
#ifndef DEFINE_H
#define DEFINE_H

#include <vector>
#include <math.h>
// #include <math.h>

const double PI = 3.14159;
// #define PI M_PI

const double D2R = PI/180.0;
const double R2D = 180.0/PI;

const double XY_GRID_RESOLUTION = 0.25;  //[m]
const double YAW_GRID_RESOLUTION = 5.0*D2R;  //[rad]
const double GOAL_TYAW_TH = 5.0*D2R;  //[rad] threshold for determining whether goal is reached
const double MOTION_RESOLUTION = 0.25; //[m] path interporate resolution
const double N_STEER = 100.0; // number of steer command
const int SKIP_COLLISION_CHECK = 4; // skip number for collision check
const double REED_STEER_CONST =20.0*D2R; // Reed model constant steer angle
 
const double SB_COST = 20.0;  // switch back penalty cost
const double BACK_COST = 10.0; // backward penalty cost
const double STEER_CHANGE_COST = 10.0;  // steer angle change penalty cost
const double STEER_COST = 10.0;  // steer angle change penalty cost
const double JACKKNIF_COST = 0.0;  // Jackknif cost
const double H_COST = 1;  // Heuristic cost
 
// Vehicle parameter
const double VR = 0.4;  // !!vehicle radius
const double WB = 0.3;//2.89;  //!![m] wheel base: rear to front steer
const double LT = 0.01;//8.0;  //[m] rear to trailer wheel
const double W = 0.3;//1.9;  //!![m] width of vehicle
const double LF = 0.35;  //!![m] distance from rear to vehicle front end of vehicle
const double LB = 0.275;  //!![m] distance from rear to vehicle back end of vehicle
const double LTF = 0.05;//1.0;  //[m] distance from rear to vehicle front end of trailer
const double LTB = 0.1;//9.0;  //[m] distance from rear to vehicle back end of trailer
const double MAX_STEER = 20.0*D2R;  //!![rad] maximum steering angle 70 degree 
const double TR = 0.5;  // Tire radius [m] for plot
const double TW = 1.0;  // Tire width [m] for plot



// for collision check
// const double WBUBBLE_DIST = 3.5; //distance from rear and the center of whole bubble
// const double WBUBBLE_R = 10.0; // whole bubble radius
// const double B = 4.45; // distance from rear to vehicle back end
// const double C = 11.54; // distance from rear to vehicle front end
// const double I = 8.55; // width of vehicle
// const double VRX = [C, C, -B, -B, C ]
// const double VRY = [-I/2.0, I/2.0, I/2.0, -I/2.0, -I/2.0]

const std::vector<double> VRXT = {LTF, LTF, -LTB, -LTB, LTF};
const std::vector<double> VRYT = {-W/2.0, W/2.0, W/2.0, -W/2.0, -W/2.0};

// bubble parameter
const double DT = (LTF + LTB)/2.0 - LTB;
// const double DTR = (LTF + LTB)/2.0 + 0.3;
const double DTR = sqrt(pow((LTF + LTB)/2.0, 2) + pow(W/2, 2))+0.5;

const std::vector<double> VRXF = {LF, LF, -LB, -LB, LF};
const std::vector<double> VRYF = {-W/2.0, W/2.0, W/2.0, -W/2.0, -W/2.0};

// bubble parameter
const double DF = (LF + LB)/2.0 - LB;
// const double DFR = (LF + LB)/2.0 + 0.3;
const double DFR = sqrt(pow((LF + LB)/2.0, 2) + pow(W/2, 2)) + VR;


#endif
