// Node class
#ifndef NODE_RRT_H
#define NODE_RRT_H

#include <iostream>
#include <vector>
#include <cmath>


class Node;

class Node{

    public:
        Node() {};
        Node(double xind, double yind): Node(xind, yind, 0.0, true, NULL){}; // BY default, the yaw angle is 0
        Node(double xind, double yind, double yawind): Node(xind, yind, yawind, true, NULL){};
        Node(double xind, double yind, double yaw1, bool direction1, Node* parent1){
            x = xind;
            y = yind;
            yaw = yaw1;
            direction = direction1;
            parent = parent1;
        };

    // public:
        double x; // x position
        double y; // y position
        double yaw; //heading angle
        bool direction; // True: + direction
        Node *parent; //the parent node

};


#endif 
