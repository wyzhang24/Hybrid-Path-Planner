#include "local_planner/dwa/dwa.h"
#include <ctime>
//#include "global_planner/hybrid_a_star/pathfinder_hybrid_astar.hpp"
#include "global_planner/rrt/rrt.hpp"
#include <string>
int main(){


  Obstacle ob({ });


  // // // Parallel parking
  // State x({-2.5, 4, PI/2, 0.0, 0.0}); // heading angle e [-PI, PI]
  // //Point goal({{6, 7}});
  // State end({-0.75, 1.4, PI/2, 0.0, 0.0});
  // Point goal({end[0], end[1]});
  // double count0 = -1.5;
  // for(int i=0; i < 15; ++i){
  //     ob.push_back({count0, 0.0});
  //     ob.push_back({count0, 2.8});
  //     //ob.push_back({-3.5, count0});
  //     count0 += 0.1;
  // }; 

  // double count1 = -4;
  // for (int i=0; i < 100; ++i){
  //     //ob.push_back({count1, 6.2});
  //     ob.push_back({0, count1});
  //     ob.push_back({-3.5, count1});
  //     if(count1 < 0 || count1 > 2.8)
  //       ob.push_back({-1.5, count1});
  //     count1 += 0.1;
  // };

  // double count2 = -3.5;
  // for(int i=0; i < 35; ++i){
  //   ob.push_back({count2, -4.0});
  //   ob.push_back({count2, 6.0});
  //   count2 += 0.1;
  // }


  // // reverse parking
  State x({-3.5, -3, PI/2, 0.0, 0.0}); // heading angle e [-PI, PI]
  //Point goal({{6, 7}});
  State end({-1, 1.2, PI, 0.0, 0.0});
  Point goal({end[0], end[1]});
  double count0 = -2.2;
  for(int i=0; i < 22; ++i){
      ob.push_back({count0, 0.0});
      ob.push_back({count0, 2.4});
      //ob.push_back({-3.5, count0});
      count0 += 0.1;
  }; 

  double count1 = -4;
  for (int i=0; i < 100; ++i){
      //ob.push_back({count1, 6.2});
      ob.push_back({0, count1});
      ob.push_back({-5, count1});
      if(count1 < 0 || count1 > 2.4)
        ob.push_back({-2.2, count1});
      count1 += 0.1;
  };

  double count2 = -5;
  for(int i =0; i < 50; ++i){
    ob.push_back({count2, -4});
    ob.push_back({count2, 6});
    count2 += 0.1;
  }

  // // drive along the road
  // State x({-2.5, 0, 0, 0.0, 0.0}); // heading angle e [-PI, PI]
  // State end({3.5,0.0, 0, 0.0, 0.0});
  // Point goal({end[0], end[1]});


  // double count0 = -8;
  // for(int i=0; i < 160; ++i){
  //     ob.push_back({count0, 2.0});
  //     ob.push_back({count0, -2.0});
  //     //ob.push_back({-3.5, count0});
  //     count0 += 0.1;
  // }; 

  // double count1 = -2;
  // for (int i=0; i < 40; ++i){
  //     //ob.push_back({count1, 6.2});
  //     ob.push_back({-8, count1});
  //     ob.push_back({8, count1});
  //     count1 += 0.1;
  // };


  // complex environment
  // State x({-1.5, 0, 0, PI, 0.0}); // heading angle e [-PI, PI]
  // State end({2.5,0.0, 0, 0.0, 0.0});
  // Point goal({end[0], end[1]});


  // double count0 = -4;
  // for(int i=0; i < 80; ++i){
  //     ob.push_back({0.0, count0});
  //     ob.push_back({-5.0, count0});
  //     //ob.push_back({-3.5, count0});
  //     count0 += 0.1;
  // }; 

  // double count1 = -2.8;
  // for (int i=0; i < 28; ++i){
  //     //ob.push_back({count1, 6.2});
  //     ob.push_back({count1, 1.0});
  //     ob.push_back({count1, -1.0});
  //     ob.push_back({count1, -4.0});
  //     count1 += 0.1;
  // };

  // double count2 = -4.5;
  // for(int i=0; i < 45; ++i){
  //   ob.push_back({count2, 4});
  //   count2 += 0.1;
  // }


  // hybrid a star
  // vector<double> oxx;
  // vector<double> oyy;

  // for(int i=0; i < ob.size(); ++i){
  //   oxx.push_back(ob[i][0]);
  //   oyy.push_back(ob[i][1]);
  // }

  // Map test_map(-20, -20, 20, 20, &oxx, &oyy);

  // Point start = {x[0], x[1]};
  // clock_t beginTime = clock();
  // Path_Final * final_path = calc_hybrid_astar_path(x[0], x[1], x[2], x[2], end[0], end[1], end[2], end[2], &test_map);
  // clock_t endTime = clock();
  // double elapsed_time = double(endTime - beginTime) / CLOCKS_PER_SEC;
  // cout << "elapsed time = " << elapsed_time << endl;
  // int size = final_path->x.size();
  // vector<vector<double>> newline;
  // ofstream out("hybrid_a_star_path.txt");
  // for(int i = 0; i < size; ++i){
  //   newline.push_back({final_path->x[i], final_path->y[i], final_path->yaw[i], final_path->direction[i]});
  //   out << newline.back()[0] << " " << newline.back()[1] << " " << newline.back()[2] << " " << newline.back()[3] << endl;
  // }
  // out.close();

 


  // rrt
  clock_t begin_rrt_time = clock();
  auto newline = execute_rrt(ob, x, end);
  clock_t end_rrt_time = clock();
  double elapsed_time = double(end_rrt_time - begin_rrt_time) / CLOCKS_PER_SEC;
  cout << "elapsed time for rrt: " << elapsed_time << "s" << endl;





  cout << "im here" <<endl;


  Control u({{0.0, 0.0}});
  double startX = x[0];
  double startY = x[1];
  vector<bool> direction_in;
  vector<pair<double, double>> line;
  for(auto& i : newline) {
    line.push_back(make_pair(i[0], i[1]));
    if(i.back() > 0) direction_in.push_back(true);
    else direction_in.push_back(false);
  }
  
  Config config(startX, startY, direction_in, line);
  Traj traj;
  traj.push_back(x);

  bool terminal = false;

  cv::namedWindow("dwa", cv::WINDOW_NORMAL);
  int count = 0;

  for(int i=0; i<1000 && !terminal; i++){
    Traj ltraj = dwa_control(x, u, config, goal, ob);
    x = motion(x, u, config.dt);
    // cout << "v = " << u[0] << " w = " << u[1] << endl;
    cout << "current index = " << config.curIdx << endl;
    traj.push_back(x);


    // visualization
    cv::Mat bg(3500,3500, CV_8UC3, cv::Scalar(255,255,255));
    cv::circle(bg, cv_offset(goal[0], goal[1], bg.cols, bg.rows),
               30, cv::Scalar(255,0,0), 5);
    for(unsigned int j=0; j<ob.size(); j++){
      cv::circle(bg, cv_offset(ob[j][0], ob[j][1], bg.cols, bg.rows),
                 20, cv::Scalar(0,0,0), -1);
    }

    for(unsigned int j=0; j < line.size(); j++){
      cv::circle(bg, cv_offset(line[j].first, line[j].second, bg.cols, bg.rows),
                 20, cv::Scalar(255,0,0), -1);
    }

    for(unsigned int j=0; j<ltraj.size(); j++){
      cv::circle(bg, cv_offset(ltraj[j][0], ltraj[j][1], bg.cols, bg.rows),
                 7, cv::Scalar(0,255,0), -1);
    }
    cv::circle(bg, cv_offset(x[0], x[1], bg.cols, bg.rows),
               30, cv::Scalar(0,0,255), 5);


    cv::arrowedLine(
      bg,
      cv_offset(x[0], x[1], bg.cols, bg.rows),
      cv_offset(x[0] + std::cos(x[2]), x[1] + std::sin(x[2]), bg.cols, bg.rows),
      cv::Scalar(255,0,255),
      7);
    cv::line(bg, cv_offset(start[0], start[1], bg.cols, bg.rows),
      cv_offset(goal[0], goal[1], bg.cols, bg.rows),  cv::Scalar(255,0,255), 7);

    if (std::sqrt(std::pow((x[0] - goal[0]), 2) + std::pow((x[1] - goal[1]), 2)) <= config.robot_radius){
      terminal = true;
      for(unsigned int j=0; j<traj.size(); j++){
        cv::circle(bg, cv_offset(traj[j][0], traj[j][1], bg.cols, bg.rows),
                    7, cv::Scalar(0,0,255), -1);
      }
    }


    cv::imshow("dwa", bg);
    cv::waitKey(5);

    // std::string int_count = std::to_string(count);
    // cv::imwrite("./pngs/"+std::string(5-int_count.length(), '0').append(int_count)+".png", bg);

    count++;
  }
}
