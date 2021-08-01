#pragma once
#include "road_network.h"
#include "solver.h"

class physarum_solver : public solver
{
private:
    double initial_flux;
    double initial_conductivity;
    double shrink_rate;
    std::vector<double> node_pressures;
    std::vector<std::vector<double>> conductivity_matrix;
    std::vector<std::vector<double>> flow_matrix;

    void update_pressures();
    void update_flow();
    void update_conductivities();

public:
    physarum_solver(road_network rn, int vehicle_limit, double vehicle_capacity, double initial_flux, double initial_conductivity, double shrink_rate);
    void solve();
};
