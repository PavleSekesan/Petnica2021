#include "road_network.h"
#include <iostream>

road_network::road_network()
{
    this->nodes = std::vector<node>();
    this->distance_matrix = std::vector<std::vector<double>>();
}

road_network::road_network(std::vector<node> nodes)
{
    this->nodes = nodes;
    this->distance_matrix = std::vector<std::vector<double>>(nodes.size(), std::vector<double>(nodes.size()));
    for(int i = 0; i < nodes.size(); i++)
    {
        for(int j = 0; j < nodes.size(); j++)
        {
            distance_matrix[i][j] = nodes[i].distance(nodes[j]);
            std::cout << distance_matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

road_network::road_network(std::vector<std::vector<double>> distance_matrix)
{
    this->distance_matrix = distance_matrix;
}

// NOT WORKING
void road_network::add_node(node p)
{
    nodes.push_back(p);

    // Update distance matrix
    std::vector<double> new_distances(nodes.size());
    for(int i = 0; i < nodes.size(); i++)
        new_distances[i] = nodes.back().distance(nodes[i]);
    for(int i = 0 ; i < nodes.size() - 1; i++)
        distance_matrix[i].push_back(new_distances[i]);
    distance_matrix.push_back(std::vector<double>(nodes.size()));
    for(int i = 0; i < nodes.size(); i++)
        distance_matrix[nodes.size() - 1][i] = new_distances[i];
}

node road_network::get_node(int index)
{
    return nodes[index];
}

double road_network::get_distance(int node1, int node2)
{
    return distance_matrix[node1][node2];
}

int road_network::size()
{
    return distance_matrix.size();
}
