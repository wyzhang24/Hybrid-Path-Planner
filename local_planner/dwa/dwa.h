#ifndef DWA_H
#define DWA_H
#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <queue>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../../def_all.hpp"
using namespace std;
using Traj = std::vector<std::array<double, 5>>;
using Obstacle = std::vector<std::array<double, 2>>;
using State = std::array<double, 5>;
using Window = std::array<double, 4>;
using Point = std::array<double, 2>;
using Control = std::array<double, 2>;
double factor = 1;
class Config{
public:
  double max_speed = 1.0;
  double min_speed = - 0.5;
  double max_yawrate = 40.0 * PI / 180.0;
  double max_accel = 0.6;
  double robot_radius = 0.5;//1.0;
  double max_dyawrate = 40.0 * PI / 180.0;
  double max_steering = 60.0 * PI / 180.0; //
  double coeff = 1.0;
  double v_reso = 0.01;
  double yawrate_reso = 1 * PI / 180.0;

  double dt = 0.1;
  double predict_time = 2.0;

  // modify the cost gains here
  double to_goal_cost_gain = 0; //0.5;
  double speed_cost_gain = 1;
  double ob_cost_gain = 0.1;
  double grid_cost_gain = 0.6; //0.002;
  double oscillation_cost_gain = 0.04;

  vector<pair<double, double>> path; // double path
  vector<pair<int, int>> path_; // int/grid path 
  vector<pair<bool, vector<pair<double, double>>>> pathVec;
  vector<bool> direction;
  bool pathDir;
  int pathIdx = 0;
  int width_;
  int height_;
  double reverseCost = 1;
  double startPointX;
  double startPointY;
  Point initialPoint;
  double inflateCoeff = 2;
  int curIdx = 0;

  // have a vector<bool> direction input
  Config():Config(0, 0, vector<bool>{}, vector<pair<double, double>>{{}}, 1001, 1001) {}
  Config(int x, int y, vector<bool> direction_in, vector<pair<double, double>> path_in): Config(x, y, direction_in, path_in, 1001, 1001) {}
  Config(int x, int y, vector<bool> direction_in, vector<pair<double, double>> path_in, int width, int height): startPointX(x), startPointY(y), direction(direction_in), path(path_in), width_(width), height_(height) {
    initialPoint = {startPointX, startPointY};
    max_yawrate = max_steering;
    path_ = doublePathToIntPath();
    segmentPath();
  }
  // segment the path according to the direction
  // direction = true if moving forward, false otherwiseg++ --std=c++11 main.cpp `pkg-config --libs opencv` -o test

  void segmentPath() {
    pair<bool, vector<pair<double, double>>> tmpPath(direction[0], vector<pair<double, double>>{path[0]});
    direction[0] = direction[1];
    direction.back() = direction[direction.size() - 2];
    pathDir = direction[1];
    for(int i = 1; i < path.size(); ++i) {
      if(direction[i] != direction[i - 1]) {
        pathVec.push_back(tmpPath);
        tmpPath = pair<bool, vector<pair<double, double>>>(direction[i], vector<pair<double, double>>{path[i]});
      }
      else {
        tmpPath.second.push_back(path[i]);
        // if(i == path.size() - 1) {
        //   pathVec.push_back(tmpPath);
        // }
      }
    }
    pathVec.push_back(tmpPath);
    for(int i = 0; i < pathVec.size(); ++i) {
      for(int j = 0; j < pathVec[i].second.size(); ++j) {
        cout << pathVec[i].second[j].first << " " << pathVec[i].second[j].second << endl;
      }
      // cout << pathVec[i].second.size() << endl;
    }
  }

  vector<pair<int, int>> doublePathToIntPath() {
    vector<pair<int, int>> path_;
    for(auto i : path) {
      pair<int, int> nodePoint = doublePointToIntPoint(i, make_pair(startPointX, startPointY));
      if(path_.empty() || nodePoint.first != path_.back().first)
      path_.emplace_back(nodePoint);
    }
    return path_;
  }
  pair<int, int> doublePointToIntPoint(Point curPoint, Point startPoint) {
    int xOffset = (curPoint[0] - startPoint[0]) / XY_GRID_RESOLUTION;
    int yOffset = (curPoint[1] - startPoint[1]) / XY_GRID_RESOLUTION;
    int widthOffset = width_ / 2;
    int heightOffset = height_ / 2;
    return make_pair(xOffset + widthOffset, yOffset + heightOffset);
  }
  pair<int, int> doublePointToIntPoint(pair<int, int> curPoint, pair<int, int> startPoint) {
    int xOffset = (curPoint.first - startPoint.first) / XY_GRID_RESOLUTION;
    int yOffset = (curPoint.second - startPoint.second) / XY_GRID_RESOLUTION;
    int widthOffset = width_ / 2;
    int heightOffset = height_ / 2;
    return make_pair(xOffset + widthOffset, yOffset + heightOffset);
  }
};

/*
  State (traj is a set of states):
  0: x
  1: y
  2: yaw
  3: v
  4: dyaw // angular velocity
  Control:
  0: v
  1: dyaw // here for car-like model, set dyaw to be the steering angle
*/
double distCalc(double x0, double y0, double x1, double y1) {
  return pow(x0 - x1, 2) + pow(y0 - y1, 2);
}

State motion(State x, Control u, double dt) {
  x[2] += factor * u[0] * u[1] * dt;
  //x[2] e [-pi, pi]
  if(x[2] > PI) {
    x[2] = x[2] - 2 * PI;
  }
  else if(x[2] < - PI) {
    x[2] = 2 * PI + x[2];
  }
  x[0] += u[0] * std::cos(x[2]) * dt;
  x[1] += u[0] * std::sin(x[2]) * dt;
  x[3] = u[0];
  x[4] = factor * u[1] * u[0]; // modify it for different vehicle models if necessary, here x[4] is the angular velocity derived from 
                               // steerting and velocity
  return x;
}

Window calc_dynamic_window(State x, Config config) {

  return {{
    std::max((x[3] - config.max_accel * config.dt), config.min_speed),
    std::min((x[3] + config.max_accel * config.dt), config.max_speed),
    //std::max((x[4] - config.max_dyawrate * config.dt), -config.max_yawrate),
    //std::min((x[4] + config.max_dyawrate * config.dt), config.max_yawrate)
    - config.max_steering, config.max_steering
  }};
}

Traj calc_trajectory(State x, double v, double y, Config config) {

  Traj traj;
  traj.push_back(x);
  double time = 0.0;
  while (time <= config.predict_time){
    x = motion(x, std::array<double, 2>{{v, y}}, config.dt);
    traj.push_back(x);
    time += config.dt;
  }
  return traj;
}

double calc_obstacle_cost(Traj traj, Obstacle ob, Config config) {
  // calc obstacle cost inf: collision, 0:free
  int skip_n = 1;
  double minr = std::numeric_limits<double>::max();

  for (unsigned int ii=0; ii<traj.size(); ii+=skip_n){
    for (unsigned int i=0; i< ob.size(); i+=skip_n){
      double ox = ob[i][0];
      double oy = ob[i][1];
      double dx = traj[ii][0] - ox;
      double dy = traj[ii][1] - oy;

      double r = std::sqrt(dx*dx + dy*dy);
      if (r <= config.robot_radius){
          return std::numeric_limits<double>::max();
      }

      if (minr >= r){
          minr = r;
      }
    }
  }
  // return 0;
  // if(minr > config.inflateCoeff * config.robot_radius) return 0;
  return 1.0 / minr;
}

double calc_to_goal_cost(Traj traj, Point goal, Config config) {
/*
  double goal_magnitude = std::sqrt(goal[0]*goal[0] + goal[1]*goal[1]);
  double traj_magnitude = std::sqrt(std::pow(traj.back()[0], 2) + std::pow(traj.back()[1], 2));
  double dot_product = (goal[0] * traj.back()[0]) + (goal[1] * traj.back()[1]);
  double error = dot_product / (goal_magnitude * traj_magnitude);
  double error_angle = std::acos(error);
  double cost = config.to_goal_cost_gain * error_angle;
*/
  // calc angle difference version
  double dx = goal[0] - traj.back()[0];
  double dy = goal[1] - traj.back()[1];
  double error_angle = atan2(dy, dx);
  // cout << "error angle = " << error_angle << endl;
  // double cost = fabs(error_angle - traj.back()[2]);
  int dist =  (((error_angle - traj.back()[2]) * 180 / PI) + 360);
  // cout << "traj angle = " << traj.back()[2] << endl;
  dist = dist % 360;
  if (dist > 180) dist = 360 - dist;
  // dist -- [0, pi]
  double cost = traj.back()[3] > 0 ? dist * PI / 180 : (180 - dist) * PI / 180;
  // cost = dist * PI / 180;
  return 0.15 * cost;
  
  // calc dist version
  // Point final_point = {traj.back()[0], traj.back()[1]};
  // return distCalc(final_point[0], final_point[1], goal[0], goal[1]);
}

// double calc_grid_cost(Traj traj, Config config) {
//   Point point = {traj.back()[0], traj.back()[1]};
//   int neighbor[8][2] = {
//     {1, 0},
//     {-1, 0},
//     {0, 1},
//     {0, -1},
//     {1, 1},
//     {1, -1},
//     {-1, 1},
//     {-1, -1}
//   };
//   pair<int, int> int_point = config.doublePointToIntPoint(point, config.initialPoint);
//   // int_point.first = point[0] / XY_GRID_RESOLUTION;
//   // int_point.second = point[1] / XY_GRID_RESOLUTION;
//   queue<pair<int, int>> q;
//   // config.path_; // the path which stores integer points.
//   vector<bool> visitList(config.height_ * config.width_,  false);
//   // bool visitList[config.height_][config.width_] = {{false}};
//   visitList[int_point.first * config.width_ + int_point.second] = true;
//   // cout << int_point.first << " " << int_point.second << endl;
//   // visitList[int_point.first * config.width_ + int_point.second] = true;
//   q.push(int_point);
//   auto curPoint =  int_point;
//   /*
//   pair<int, int> closest_point;
//   double min_dist = INFINITY;
//   for(auto i : config.path_) {
//     double tmp_dist = distCalc(int_point.first, int_point.second, i.first, i.second);
//     if(tmp_dist < min_dist) {
//       min_dist = tmp_dist;
//       closest_point = i;
//     }
//   }
//   return min_dist;
//   */
//   // start BFS to find closest point
//   while(find(config.path_.begin(), config.path_.end(), curPoint) == config.path_.end() && !q.empty()) {
//     // cout << "in loop" << endl;
//     for(int i = 0; i < 8; ++i) {
//       auto tmpPoint = curPoint;
//       tmpPoint.first += neighbor[i][0];
//       tmpPoint.second += neighbor[i][1];
//       // cout << tmpPoint.first << " " << tmpPoint.second << endl;
//       if(tmpPoint.first < config.height_
//       && tmpPoint.first >= 0
//       && tmpPoint.second < config.width_
//       && tmpPoint.second >= 0
//       && !visitList[tmpPoint.first * config.width_ + tmpPoint.second]) {
//         visitList[tmpPoint.first * config.width_ + tmpPoint.second] = true;
//         q.push(tmpPoint);
//       }
//     }
//     curPoint = q.front();
//     q.pop();
//   }
//   // cout << curPoint.first << " " << curPoint.second << " " << int_point.first << " " << int_point.second << endl;
//   return distCalc(curPoint.first, curPoint.second, int_point.first, int_point.second);
// }

double calc_grid_cost(Traj traj, Config& config) {
  Point lastPoint = {traj.back()[0], traj.back()[1]};
  Point firstPoint  = {traj.front()[0], traj.front()[1]};
  double minDist = INT_MAX;
  int idx = 0;
  // find the min dist and the closest point index on path
  for(int i = config.curIdx; i < config.path.size(); ++i) {
    double curDist = distCalc(firstPoint[0], firstPoint[1], config.path[i].first, config.path[i].second);
    if(minDist > curDist) {
      minDist = curDist;
      idx = i;
    }
  }
  //calculate the distance cost of all points on trajectory to the next 3 poinits
  double cost = 0;
  // double factor = 1.0 / traj.size();
  // for(auto& i : traj) {
  //   cost += distCalc(i[0], i[1], config.path[idx].first, config.path[idx].second);
  //   cost += idx + 1 < config.path.size() ? distCalc(i[0], i[1], config.path[idx + 1].first, config.path[idx + 1].second)
  //    : distCalc(i[0], i[1], config.path[idx].first, config.path[idx].second);
  //   cost += idx + 2 < config.path.size() ? distCalc(i[0], i[1], config.path[idx + 2].first, config.path[idx + 2].second)
  //    : distCalc(i[0], i[1], config.path[idx].first, config.path[idx].second);
  // }
  // cout << "size = " << config.path.size() << endl;
  // for(auto& i : config.path) {
  //   cout << i.first << " " << i.second << endl;
  // }
  config.curIdx = max(config.curIdx, idx);
  Point goalPoint = idx + 5 >= config.path.size() ? Point{config.path.back().first, config.path.back().second}
   : Point{config.path.at(idx + 5).first, config.path.at(idx + 5).second};
  // int index = idx + 3 >= config.path.size() ? config.path.size() - 1 : idx + 3;
  // cout << "goal point is " << index << endl;
  cost = calc_to_goal_cost(traj, goalPoint, config);
  Point secPoint = idx + 1 >= config.path.size() ? Point{config.path.back().first, config.path.back().second}
   : Point{config.path.at(idx + 1).first, config.path.at(idx + 1).second};
  cost += distCalc(traj[0][0], traj[0][1], secPoint[0], secPoint[1]);
  return cost;
}

// double calc_grid_cost(Traj traj, Config config) {
//   Point lastPoint = {traj.back()[0], traj.back()[1]};
//   Point firstPoint  = {traj.front()[0], traj.front()[1]};
//   double cost = 0;
//   for(auto& pOnTraj : traj) {
//     double minDist = std::numeric_limits<double>::max();
//     for(auto& pOnPath : config.path) {
//       double dist = distCalc(pOnTraj[0], pOnTraj[1], pOnPath.first, pOnPath.second);
//       minDist = min(minDist, dist);
//     }
//     cost += 5.0 / traj.size() * minDist;
//   }
//   return cost;
// }

double calc_oscillation_cost(State x, Traj traj, Config config) {
  // avoid large linear and augular acceleration, especially augular one
  // also, pay penalty to zero velocity
  // velocity.first = linear velocity, velocity.second = angular velocity
  pair<double, double> cur_velocity = make_pair(traj.back()[3], traj.back()[4]);
  pair<double, double> prev_velocity = make_pair(x[3], x[4]);
  // x[4] = factor * u[1] * u[0];
  prev_velocity.second = prev_velocity.first == 0 ? 0 : x[4] / (factor * prev_velocity.first);
  // v
  // dyaw
  // check v and dyaw (rad/s)
  // double avoid_zero_speed_cost = fabs(2.0 / cur_velocity.first);
  // cout << "cur velocity = " << avoid_zero_speed_cost << endl;
  double oscillate_cost = fabs(cur_velocity.second - prev_velocity.second);
  oscillate_cost += fabs(x[2]); // penalize large steering
  return oscillate_cost;
}

double calc_speed_cost(Traj traj, Config config) {
  double speed = traj.back()[3];
  double cost = 0;
  // cost += speed < 0 ? fabs(config.min_speed - speed) : config.max_speed - fabs(speed);
  // cost += config.max_speed - fabs(speed);
  if((speed < 0 && config.pathDir) || (speed > 0 && !config.pathDir)) {
    cost = fabs(speed);
    int intCost = 100 * cost;
    int factor_ = intCost / 10 + 1;
    cost = factor_ * config.reverseCost;
  }
  else {
    cost = speed < 0 ? fabs(config.min_speed - speed) : config.max_speed - fabs(speed);
  }
  return cost;
}

Traj calc_final_input(
  State x, Control& u,
  Window dw, Config& config, Point goal,
  std::vector<std::array<double, 2>>ob) {

    double min_cost = 10000.0;
    Control min_u = u;
    min_u[0] = 0.0;
    Traj best_traj;
    double to_goal_cost_min, speed_cost_min, ob_cost_min, grid_cost_min, oscillation_cost_min;
    // evalucate all trajectory with sampled input in dynamic window
    for (double v=dw[0]; v<=dw[1]; v+=config.v_reso){
      for (double y=dw[2]; y<=dw[3]; y+=config.yawrate_reso){

        Traj traj = calc_trajectory(x, v, y, config);

        double to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(traj, goal, config);
        double speed_cost = config.speed_cost_gain * calc_speed_cost(traj, config);
        double ob_cost = config.ob_cost_gain * calc_obstacle_cost(traj, ob, config);
        double grid_cost = config.grid_cost_gain * calc_grid_cost(traj, config);
        double oscillation_cost = config.oscillation_cost_gain * calc_oscillation_cost(x, traj, config);
        double final_cost = speed_cost + ob_cost + grid_cost + oscillation_cost;

        if (min_cost >= final_cost){
          min_cost = final_cost;
          min_u = Control{{v, y}};
          best_traj = traj;
          to_goal_cost_min = to_goal_cost;
          speed_cost_min = speed_cost;
          ob_cost_min = ob_cost;
          grid_cost_min = grid_cost;
          oscillation_cost_min = oscillation_cost;
        }
      }
    }
    u = min_u;
    cout << "goal_cost = " << to_goal_cost_min << " speed_cost = " << speed_cost_min << " ob cost = " << ob_cost_min <<
    " grid_cost = " << grid_cost_min << " osc cost = " << oscillation_cost_min << endl;
    cout << "path direction = " << config.pathDir << endl;
    return best_traj;
}

Traj dwa_control(State x, Control & u, Config & config,
  Point goal, Obstacle ob){
    // # Dynamic Window control
    // if the current path has been traversed, move on to the next path
    config.path = config.pathVec[config.pathIdx].second;
    if(sqrt(distCalc(x[0], x[1], config.path.back().first, config.path.back().second)) < 0.2 * config.robot_radius) {
      if(config.pathIdx + 1 < config.pathVec.size()) {
        config.path = config.pathVec[++config.pathIdx].second;
        config.pathDir = config.pathVec[config.pathIdx].first;
      }
    }
    // cout << "goal idx = " << config.path.size() << endl;
    Window dw = calc_dynamic_window(x, config);
    Traj traj = calc_final_input(x, u, dw, config, goal, ob);

    return u, traj;
  }

cv::Point2i cv_offset(
    double x, double y, int image_width=2000, int image_height=2000){
  cv::Point2i output;
  output.x = int(x * 100) + image_width/2;
  output.y = image_height - int(y * 100) - image_height/3;
  return output;
}

vector<pair<double, double>> extend_line(Point current, Point goal) {
    double dist = sqrt(pow(current[0] - goal[0], 2) + pow(current[1] - goal[1], 2));
    int num = dist / XY_GRID_RESOLUTION;
    double delta_x = (goal[0] - current[0]) / num;
    double delta_y = (goal[1] - current[1]) / num;
    vector<pair<double, double>> line;
    pair<double, double> prev_point = {current[0], current[1]};
    for(int i = 1; i <= num; ++i) {
        pair<double, double> waypoint = {prev_point.first + delta_x, prev_point.second + delta_y};
        prev_point = waypoint;
        line.push_back(waypoint);
    }
    return line;
}

#endif
