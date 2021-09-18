#include <iostream>
#include <iomanip>
#include "road_network.h"
#include "physarum_solver.h"
#include "nsga_solver.h"
#include "random_gen.h"
#include <fstream>
#include <math.h>
#include <random>
#include <chrono>
#include <string>

int main()
{
    std::string seed_str = "jajasti";
    std::seed_seq seed(seed_str.begin(), seed_str.end());
    random_gen::rng = std::mt19937(seed);
    std::ifstream input("city.txt");
    int n; input >> n;
    std::vector<node> nodes;
    for (int i = 0; i < n; i++)
    {
        double x, y, pop; input >> y >> x >> pop;
        nodes.push_back(node(x, y, pop));
    }
    road_network rn = road_network(nodes);

    bool solve_physarum = true;
    bool solve_nsga = true;
    if (solve_physarum)
    {
        std::ofstream physarum_solution("C:/Users/Pavle/Desktop/Petnica2021 pomocne skripte/physarum.txt");
        physarum_solver ps = physarum_solver(rn);
        ps.set_stream(physarum_solution);
        ps.solve();
    }
    if (solve_nsga)
    {
        std::ofstream nsga_solution("C:/Users/Pavle/Desktop/Petnica2021 pomocne skripte/nsga.txt");
        nsga_solver ns = nsga_solver(rn);
        ns.set_stream(nsga_solution);
        ns.solve();
    }
}
