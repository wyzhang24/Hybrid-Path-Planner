#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <time.h>
#include <math.h>
#include <cstring>
#include "node_rrt.hpp"
#include "RRT_parameters.hpp"
#include "tree.hpp"


using namespace std;

class yaw_ability{
    public:
    yaw_ability(bool flag, double angle){
        inrange_flag = flag;
        steering = angle;
    };
    public:
        bool inrange_flag;
        double steering;
};

class Extend_return{
    public:
        Extend_return(){};
        Extend_return(string s, Node* q_new){
            res_string = s;
            q_node = q_new;
        }

    public:
        string res_string;
        Node* q_node;
};

double wrap2pi(double angle){
    while(angle > PI){
        angle -= 2*PI;
    }
    while(angle < -PI){
        angle += 2*PI;
    }
    //cout << angle<< endl;
    return angle;
};

double Cal_Dist(Node *q1, Node *q2){
    double dist;
    dist = sqrt(pow(q1->x - q2->x, 2) + pow(q1->y - q2->y, 2));
    return dist;
};

double Cal_Dist_2_obstacle(Node *q1, vector<double> obstacle){
    double dist;
    dist = sqrt(pow(q1->x - obstacle[0], 2) + pow(q1->y - obstacle[1], 2));
    return dist;
};

yaw_ability Check_Reachability(Node* q1, Node* q2, int flip_time){
    double steer;
    steer = atan2((q2->y - q1->y), (q2->x - q1->x));
    bool range_flag1 = true;
    bool range_flag2 = true; 
    bool range_flag3 = true;

    // if (abs(wrap2pi(steer - q1->yaw)) + abs(wrap2pi(steer - q2->yaw)) > 1/2*max_yaw_range) range_flag1 = false;
    // if (abs(wrap2pi(q2->yaw - steer+PI))+abs(wrap2pi(q1->yaw-steer)) > 1/2*max_yaw_range) range_flag2 = false;

    if (abs(wrap2pi(q2->yaw-steer)) > 0.5*max_yaw_range ) range_flag3 = false;

    if (abs(wrap2pi(steer - q1->yaw)) > max_yaw_range) range_flag1 = false;

    if (abs(wrap2pi(steer - q1->yaw + PI)) > max_yaw_range) range_flag2 = false;
    //if (abs(wrap2pi(abs(q2->yaw - steer)+abs(q1->yaw-steer) + PI)) > max_yaw_range) range_flag2 = false;
    // if(cnect_flag != 1){
    //     if((range_flag1 || range_flag2) && range_flag3) return yaw_ability(true, steer);
    // }
    // else{
    //     if(range_flag2 && range_flag3 && q2->direction == true) return yaw_ability(true, steer);
    // }
    if((range_flag1 || range_flag2) && range_flag3) return yaw_ability(true, steer);
    // if (range_flag1 && range_flag3 && cnect_flag != 1) return yaw_ability(true, steer);
    // if (range_flag2 && range_flag3 && cnect_flag != 1 && q2->direction) return yaw_ability(true, steer);

    return yaw_ability(false, steer);
};


bool is_incollsion(Node* node){
    for(size_t i = 0; i < obstacles.size(); ++i){
        if (Cal_Dist_2_obstacle(node, obstacles[i]) < car_radius) {
            //cout << "im here obstacle" << endl;
            return true;
        }
    }
   // cout << "not obstacle" <<endl;
    return false;
};

Node RANDOM_CONFIG(){
    //cout << rand() << endl;
    double rand_goal = (double(rand()) / RAND_MAX);
    double rand_x = (double(rand()) / RAND_MAX)* xrange_map - xrange_map/2;
    double rand_y = (double(rand()) / RAND_MAX)* yrange_map - yrange_map/2;
    double rand_angle = wrap2pi((double(rand()))/ RAND_MAX * 2* PI);
    //cout << "rand_angle:" << rand_angle << endl;
    //if(rand_goal > 0.8) return Node(goal[0], goal[1]);
    return Node(rand_x, rand_y, rand_angle);
};

Node* NEAREAST_NEIGHBOR(Node *q, Tree &T, int flip_time){
    //cout << "T_a: x: " << T.Vertices_list[0]->x << endl;

    double min_dist = 1e6;
    Node *q_nearest = NULL;
    Node* q_temp;
    //cout << "vert_size: " << T.Vertices_list.size() << endl;
    for(int i = 0; i < T.Vertices_list.size(); ++i){
       //cout << "T_a: x: " << (T.Vertices_list[i])->x << endl;

        q_temp = T.Vertices_list[i];
        //cout << q_temp->x << endl;
        //cout << "In nearneabor: "<< q_temp->x << " " << q_temp->y << " " << q_temp->parent->x << " " << q_temp->parent->y <<endl;
        yaw_ability reachability = Check_Reachability(q_temp, q, flip_time);
        if (reachability.inrange_flag == false) continue;

        double dist = Cal_Dist(q_temp, q);
        if(dist < min_dist){
            min_dist = dist;
            q_nearest = q_temp;
        }
    }
    // if(q_nearest != NULL)
    //cout << "In nearneabor: " << q_nearest->x << " " << q_nearest->y <<" " << q_nearest->yaw << endl;
    return q_nearest;
};


Node* NEW_CONFIG(Node* q_rand, Node* q_near, int flip_time, int is_extend){
    //cout<< "In new config, the randnode is: " << q_rand->x << " " << q_rand->y << " " << q_rand->yaw<<endl; 
    double norm = sqrt(pow((q_near->y - q_rand->y), 2) + pow((q_near->x-q_rand->x), 2));

    double y_direct = (q_rand->y - q_near->y)/norm;
    double x_direct = (q_rand->x - q_near->x)/norm;
    // double yaw_direct = wrap2pi(wrap2pi(q_rand->yaw-q_near->yaw)/norm*step_len);
    // cout <<"wrap to pi:" << q_rand->yaw-q_near->yaw << endl;
    double steer = atan2(y_direct, x_direct);

    Node* new_node = new Node;
    //cout << new_node->x << endl;
    new_node->x = x_direct*step_len + q_near->x;
    //cout << new_node->x << endl;
    new_node->y = y_direct*step_len + q_near->y;
    new_node->parent = q_near;
    //cout << "steer is: " << steer << " " << "anlge is: " << q_near->yaw << endl;
    if(flip_time == 1 && is_extend !=1 || flip_time != 1 && is_extend ==1){
        //cout << "steer is: " << steer << " " << "q_near is: " << q_near->yaw <<" " << q_near->x << " " << q_near->y<<endl;
        if (abs(wrap2pi(steer - q_near->yaw)) < max_yaw_range){ // the angle is larger than max_yaw, means it's back off
            new_node->yaw = wrap2pi(steer);
           
            //cout << q_near->direction << endl;
            // if(q_near->direction == true){
            //     //cout << "back off" << endl;
            //     new_node->direction = false;
            // }
            // else{
            //     new_node->direction = true;
            // }
            //new_node->direction = ~(q_near->direction); // back off
           // cout << new_node->direction << "  " << q_near->direction << endl;
        }
        else{
            //cout <<"yaw_direct is: " << yaw_direct << endl;
            new_node->yaw = wrap2pi(steer+PI);
            //new_node->direction = q_near->direction;
            // if(is_extend != 1) new_node->direction = q_near->direction;
            // else new_node->direction = q_near->direction;
        }
        if(abs(wrap2pi(atan2(-new_node->y+q_near->y, -new_node->x+q_near->x) - q_near->yaw))  > max_yaw_range){
            new_node->direction = false;
        }
        else{
            new_node->direction = true;
        }

    }  
    else{
        if (abs(wrap2pi(steer - q_near->yaw)) > max_yaw_range){ // the angle is larger than max_yaw, means it's back off
            new_node->yaw = wrap2pi(steer + PI);
           
            //cout << q_near->direction << endl;
            // if(q_near->direction == true){
            //     new_node->direction = false;
            //     //cout << "back off" << endl;
            // }
            // else{
            //     new_node->direction = true;
            // }
            //new_node->direction = ~(q_near->direction); // back off
           // cout << new_node->direction << "  " << q_near->direction << endl;
        }
        else{
            //cout <<"yaw_direct is: " << yaw_direct << endl;
            new_node->yaw = wrap2pi(steer);
            // new_node->direction = q_near->direction;
            // if(is_extend != 1) new_node->direction = q_near->direction;
            // else new_node->direction = q_near->direction;
        } 

        if(abs(wrap2pi(atan2(new_node->y-q_near->y,new_node->x-q_near->x) - q_near->yaw))  > max_yaw_range){
            new_node->direction = false;
        }
        else{
            new_node->direction = true;
        }
    }



    if (is_incollsion(new_node)) return NULL;
    //cout << new_node->x << " " << new_node->y << " " <<new_node->parent->parent << endl;
    return new_node;
};



bool is_reach_goal(Node* node1, Node* node_goal){
    if(Cal_Dist(node1, node_goal) < threshod){
        if(abs(wrap2pi(node_goal->yaw- node1->yaw)) < max_yaw_range){
            //cout << node_goal->yaw << " " << node1->yaw << endl;
            node1->yaw = node_goal->yaw;
            return true;
        }
    }
     // also can be used to cal dist between node and goal (vector dim 2) 
    return false;
};

Extend_return EXTEND(Tree &T, Node* q_rand, int* cout_back_off, int flip_time, int is_extend){
    //cout << "In extend: T_a: x: " << T.Vertices_list[0]->x << endl;

    Node* q_near = NEAREAST_NEIGHBOR(q_rand, T, flip_time);
    Extend_return res;
    if(q_near == NULL){
        //string trapped = "Trapped";
        //cout << "In extend, q_near is null trapped!" << endl;
        res.q_node = NULL;
        res.res_string = trapped; 
        return res;
    }
    //cout << "In extend: q_near is: " << q_near->x<< " " << q_near->y<< " "<< q_near->yaw << endl;

    Node* q_new = NEW_CONFIG(q_rand, q_near, flip_time, is_extend);

    if(q_new == NULL){
        //string trapped = "Trapped";
        //cout << "in extend, trapped!" << endl; 
        res.res_string = trapped;
        res.q_node = NULL;
        return res;
    }
    if(*cout_back_off < max_attemp_num && q_new->direction == false){ //cf true
        //string advanced = "Advanced";
        *cout_back_off += 1;
        res.res_string = trapped;
        res.q_node = q_new;
        return res;
    }
    else{
        *cout_back_off = 0;
    }

    T.add_vertex(q_new);
    // string reached = "Reached";
    // return reached;
    //cout << "In extend: q_new is: " << q_new->x << " " << q_new->y << " " << q_new->yaw << endl;
    //cout << T.cur_node->parent->parent << endl;
    if(is_reach_goal(q_new, q_rand)){
        //cout << "in reach, reached!" <<endl;
        //string reached = "Reached";
        res.res_string = reached;
        res.q_node = q_new;
        return res;
    }
    //cout << "advanced!" << endl;
    //string advanced = "Advanced";
    if(Cal_Dist(q_new, q_rand) < threshod &&  abs(wrap2pi(q_new->yaw- q_rand->yaw)) > max_yaw_range){
        res.res_string = trapped;
        res.q_node = NULL;
    }

    res.res_string = advanced;
    res.q_node = q_new;
    return res;
};

Extend_return CONNECT(Tree &T, Node *q, int* count_back_off, int flip_time, int is_extend){
    Extend_return res_extend;
    int count_cnnect = 0;
    do{
        res_extend = EXTEND(T, q, count_back_off, flip_time, is_extend);
        //cout << res_extend.res_string << endl;
        //cout << (res_extend.q_node)->x << " " << (res_extend.q_node)->y << " " << (res_extend.q_node)->yaw << endl;
        count_cnnect += 1;
    }while(res_extend.res_string == advanced && count_cnnect <= 20);
   //cout << "Stop connect trapped!" << endl;
    return res_extend;

};

void flip_yaw(Tree &T){
    Node* cur_node = T.cur_node;
    while(cur_node != NULL){
        cur_node->yaw = wrap2pi(cur_node->yaw+PI);
        cur_node = cur_node->parent;
    }
};



void SWAP(Tree &T_a, Tree &T_b){
    Tree temp = T_a;
    T_a = T_b;
    T_b = temp;
    //flip_yaw(T_a);
    //flip_yaw(T_b);
};

