#include <iostream>
#include <iomanip>
#include "road_network.h"
#include "physarum_solver.h"
#include <fstream>
#include <math.h>

int main()
{
    std::ifstream in("input.in");
    int v, e; in >> v >> e;
    std::vector<std::vector<double>> distance_matrix(v, std::vector<double>(v, INFINITY));
    for(int i = 0; i < e; i++)
    {
        int node1, node2; in >> node1 >> node2;
        double weight; in >> weight;
        distance_matrix[node1][node2] = weight;
        distance_matrix[node2][node1] = weight;
    }
    int source, dest; in >> source >> dest;

    std::vector<int> mapping(v);
    mapping[source] = 0;
    mapping[dest] = v - 1;
    int counter = 1;
    for(int i = 0; i < v; i++)
    {
        if (i != source && i != dest)
            mapping[i] = counter++;
    }

    std::vector<std::vector<double>> new_distance_matrix(v, std::vector<double>(v, INFINITY));
    for(int i = 0; i < v; i++)
    {
        for(int j = 0; j < v; j++)
        {
            int i1 = mapping[i];
            int j1 = mapping[j];
            new_distance_matrix[i1][j1] = distance_matrix[i][j];
        }
    }

    road_network rn = road_network(new_distance_matrix);

    physarum_solver ps = physarum_solver(rn, 1, 1000);
    ps.solve();
}
