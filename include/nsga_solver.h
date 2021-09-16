#pragma once
#include "road_network.h"
#include "solver.h"
#include "organism.h"
#include <algorithm>
#include <iostream>
#include <iomanip>

class nsga_solver : public solver
{
private:
    double mutation_rate = 0.5;
    int population_size = 100;
    int generation_count = 500;

    std::vector<organism> population;

    void generate_new_population();
    void evaluate_population();
    std::vector<std::vector<organism>> nondominated_sort();
    void crowding_distance_assignment(std::vector<organism>& front);

public:
    nsga_solver(road_network rn, int vehicle_limit, double vehicle_capacity);
    std::vector<int> solve();
    friend std::ostream& operator<<(std::ostream& os, const nsga_solver& ps);
};

