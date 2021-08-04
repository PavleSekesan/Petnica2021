#pragma once
#include <vector>
#include <iostream>
#include "node.h"

class road_network
{
private:
    std::vector<node> nodes; // node[0] is defined as the depot
    std::vector<std::vector<double>> distance_matrix;
public:
    road_network();
    road_network(std::vector<node> nodes);
    road_network(std::vector<std::vector<double>> distance_matrix);
    void add_node(node p);
    node get_node(int index);
    double get_distance(int node1, int node2);
    int size() const;
    bool is_planar();
    friend std::ostream& operator<<(std::ostream& os, const road_network& rn);
};
