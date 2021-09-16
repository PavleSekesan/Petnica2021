#include <iostream>
#include <iomanip>
#include "road_network.h"
#include "physarum_solver.h"
#include "nsga_solver.h"
#include <fstream>
#include <math.h>

int main()
{
    /*std::ifstream in("input.in");
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

    road_network rn = road_network(new_distance_matrix);*/

    freopen("city.txt", "r", stdin);
    int n; std::cin >> n;
    std::vector<node> nodes;
    for (int i = 0; i < n; i++)
    {
        double x, y, pop; std::cin >> y >> x >> pop;
        nodes.push_back(node(x, y, pop));
    }

    road_network rn = road_network(nodes);

    /*std::ofstream file("C:/Users/Pavle/Desktop/Petnica2021 pomocne skripte/conductivities.txt");
    physarum_solver ps = physarum_solver(rn, 1, 1000);
    ps.set_stream(file);
    std::vector<int> path = ps.solve();*/
    /*double path_length = 0;
    for (int i = 0; i < path.size(); i++)
    {
        std::cout << path[i] << " ";
    }
    std::cout << std::endl;
    for (int i = 0; i < path.size() - 1; i++)
    {
        path_length += rn.get_distance(path[i], path[i + 1]);
    }
    std::cout << path_length << std::endl;*/

    std::ofstream file1("C:/Users/Pavle/Desktop/Petnica2021 pomocne skripte/nsga.txt");
    nsga_solver ns = nsga_solver(rn, 1, 1000);
    ns.set_stream(file1);
    std::vector<int> path1 = ns.solve();
}
