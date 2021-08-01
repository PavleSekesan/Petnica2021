#include <iostream>
#include <iomanip>
#include "road_network.h"
#include "physarum_solver.h"

int main()
{
    int n; std::cin >> n;
    std::vector<node> nodes;
    for(int i = 0; i < n; i++)
    {
        double x, y; std::cin >> x >> y;
        nodes.push_back(node(x, y, 5.0));
    }

    road_network rn = road_network(nodes);

    physarum_solver ps = physarum_solver(rn, 1, 1000, 10, 0.5, 0.1);
    ps.solve();
}
