#pragma once
#include "road_network.h"
#include "solver.h"
#include "random_gen.h"
#include "organism.h"
#include <algorithm>
#include <iostream>
#include <iomanip>

class nsga_solver : public solver
{
private:
    void generate_new_population();
    void evaluate_population();
    std::vector<std::vector<organism>> nondominated_sort();
    void crowding_distance_assignment(std::vector<organism>& front);

    void print_generation(int generation);
    void print_solver_header();

public:
    double mutation_rate = 0.5;
    int population_size = 100;
    int generation_count = 500;

    std::vector<organism> population;

    nsga_solver(road_network rn);
    void solve();
    friend std::ostream& operator<<(std::ostream& os, const nsga_solver& ps);
};

