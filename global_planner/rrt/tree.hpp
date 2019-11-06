#ifndef TREE_H
#define TREE_H

#include <iostream>
#include <cmath>
#include <vector>
#include "node_rrt.hpp"

using namespace std;

class Tree;

class Tree{
    public:
        Tree() {};
        void add_vertex(Node* node){
            Vertices_list.push_back(node);
            cur_node = node;
        };
    public:
        vector<Node*> Vertices_list;
        Node* cur_node;

};

#endif