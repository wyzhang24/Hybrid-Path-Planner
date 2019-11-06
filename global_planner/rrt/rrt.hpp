#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <math.h>
#include <cstring>
#include "assert.h"
#include <array>
#include "node_rrt.hpp"
#include "RRT_parameters.hpp"
#include "tree.hpp"
#include "RRT_function.hpp"
#include <fstream>


using namespace std;


vector<vector<double>> execute_rrt(vector<array<double, 2>> obstacles_hk, array<double, 5> start_hk, array<double, 5> end_hk){
    Tree T_a, T_b;
    start.push_back(start_hk[0]);
    start.push_back(start_hk[1]);

    goal.push_back(end_hk[0]);
    goal.push_back(end_hk[1]);
    //cout << start[0] << start[1] << endl;
    Node q_int(start[0], start[1], start_hk[2], true, NULL);
    cout << q_int.direction << "   " << true << endl;
    Node q_goal(goal[0], goal[1], end_hk[2], true, NULL);
    T_a.add_vertex(&q_int);
    T_b.add_vertex(&q_goal);

    // Using haokun's convention
    ofstream out_ob("rrt_connect_ob.txt");
    for(size_t i=0; i < obstacles_hk.size(); i++){
        //cout << obstacles_hk[i][0] << " " << obstacles_hk[i][1] << endl;
        obstacles.push_back({obstacles_hk[i][0], obstacles_hk[i][1]});
        out_ob << obstacles_hk[i][0] << " " << obstacles_hk[i][1];
        out_ob << "\n";
    }

    // Weiyang's obstacles defination
    // double count = 0.0;
    // for(int i=0; i < 200; ++i){
    //     if(count >=120 && count < 200){ 
    //         obstacles.push_back({190, count});
    //         count += 1;
    //         continue;
    //     }
    //     obstacles.push_back({190, count});
    //     count += 1;
    // }; 

    // double count1 = 120.0;
    // for (int i=120; i <= 190; ++i){
    //     obstacles.push_back({count1, 200});
    //     obstacles.push_back({count1, 120});
    //     count1 += 1;
    // };

    // double count2 = 200;
    // for(int i = 200; i <= 600; ++i){
    //     obstacles.push_back({120, count2});
    //     count2 += 1;
    // };

    // double count3 = 500;
    // for(int i = 500; i <= 800; ++i){
    //     obstacles.push_back({500, count3});
    //     count3 += 1;
    // };

    // double count4 = 500;
    // for(int i = 500; i < 1000; ++i){
    //     obstacles.push_back({200, count4});
    //     count4 += 1;
    // };

    //cout << T.Vertices_list[0]->direction << endl;
    Extend_return res, res_connect;
    int count_back_off = 0;
    int flip_time = 1;
    for(int i = 0; i < max_iteration; ++i){
        //out << i << endl;
        Node q_rand = RANDOM_CONFIG();
        //cout <<"In main, the rand node is: " <<q_rand.x << " "<< q_rand.y<< endl;
        //cout << "T_a: x: " << T_a.Vertices_list[0]->x << endl;
        res = EXTEND(T_a, &q_rand, &count_back_off, flip_time, 1);
        if(res.res_string == trapped) {//cout << "generate node for coonect fail!"<< endl;
        continue;}
        //cout << res.res_string << endl;
        res_connect = CONNECT(T_b, res.q_node, &count_back_off, flip_time, 0); 
        if(res_connect.res_string == reached) break;
        //cout << "before swap: " << T_a.cur_node->x << " " << T_b.cur_node->x <<endl;
        SWAP(T_a, T_b);
        //cout << "after swap: " << T_a.cur_node->x << " " << T_b.cur_node->x <<endl;

        flip_time *= -1;
    }
    assert(res_connect.res_string == reached);
    //assert(res.res_string == reached);
// Using haokun's convention to generate path
//     vector<vector<double>> final_path;

//     Node *cur_node = T.cur_node;

//     while(cur_node != NULL){
//         final_path.push_back({cur_node->x, cur_node->y, cur_node->yaw, double(cur_node->direction)});
//         cout << cur_node->x << " " << cur_node->y << " " << cur_node->yaw << " " << cur_node->direction <<endl;
//         cur_node = cur_node->parent;
//     }

//     for(size_t i=0; i < obstacles.size(); ++i){
//         cout << obstacles[i][0] << " " << obstacles[i][1] << " " << 0 << " " << 0 << endl;
//     }

    vector<pair<double, double>> final_path;
    vector<vector<double>> final_path_output;
    Tree* T_a_final;
    Tree* T_b_final;

    if(flip_time == 1){
        T_a_final = &T_a;
        T_b_final = &T_b;
    }
    else{
        T_a_final = &T_b;
        T_b_final = &T_a;
    }


    Node *cur_node = (*T_a_final).cur_node;
    double dir;
    while(cur_node != NULL){
        pair<double, double> cur_point = {cur_node->x, cur_node->y};
        cout << cur_node->x << " " << cur_node->y <<" " << cur_node->direction << endl;
        final_path.push_back(cur_point);
        if(cur_node->direction) dir = 1;
        else dir = -1;
        cout << dir << endl;
        final_path_output.push_back({cur_node->x, cur_node->y, cur_node->yaw, dir});
        cur_node= cur_node->parent;
    }

    vector<pair<double, double>> final_path_reverse;
    vector<vector<double>> final_path_output_reverse;
    //cout << "get path!" <<endl;
    //cout << final_path[0].first << endl;
    //cout << final_path.size() << endl;
    for(int i = final_path.size()-1; i >=0; --i){
       // cout << i << endl;
        cout << final_path[i].first << " " << final_path[i].second << endl;
        //pair<double, double> cur_point_reverse = {final_path[i].first, final_path[i].second};
        final_path_reverse.push_back(final_path[i]);
        final_path_output_reverse.push_back(final_path_output[i]);
    }

    Node* cur_node2 = (*T_b_final).cur_node;
    double dir2;
    while(cur_node2 != NULL){
        pair<double, double> cur_point2 = {cur_node2->x, cur_node2->y};
        cout << cur_node2->x << " " << cur_node2->y << endl;
        final_path_reverse.push_back(cur_point2);
        if(cur_node2->direction) dir2 = 1;
        else dir2 = -1;
        final_path_output_reverse.push_back({cur_node2->x, cur_node2->y, cur_node2->yaw, dir2});
        cur_node2 = cur_node2->parent;
    }
    ofstream out("rrt_connect_path.txt");
    int dir3;
    for(int i = 0; i < final_path_output_reverse.size(); ++i){
        if(final_path_output_reverse[i][3]>0) dir3 = 1;
        else dir3 = 0;
        out << final_path_output_reverse[i][0] << " " << final_path_output_reverse[i][1] << " " <<final_path_output_reverse[i][2] << " " << dir3;
        out << "\n";
    }

    return final_path_output_reverse;
};