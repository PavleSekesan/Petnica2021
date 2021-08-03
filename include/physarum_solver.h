#pragma once
#include "road_network.h"
#include "solver.h"

class physarum_solver : public solver
{
private:
    double initial_flux = 1; // flow rate through the system
    double initial_conductivity = 0.5; // starting "width" of tubes
    double shrink_rate = 0.7; // how fast the tubes shrink in response to flow through a tube
    double dt = 0.001; // time step
    std::vector<double> node_pressures; // pressure for each node in system
    std::vector<std::vector<double>> conductivity_matrix; // conductivity_matrix[i][j] = conductivity between node i and node j
    std::vector<std::vector<double>> flow_matrix; // flow_matrix[i][j] = flow between node i and node j

    void update_pressures_exact(); // solves system of equations using QR decomposition to determine each node pressure (O(V^3), doesn't greatly influence solution)
    void update_pressures(); // determines node pressures approximately by plugging in pressures from previous iteration (O(V^2))
    void update_flow(); // determines flow through tube, defined as conductivity_matrix[i][j] / edge_weigth[i][j] * (pressure[i] - pressure[j])
    void update_conductivities(); // determines conductivities by numerically solving the differential equation d conductivity[i][j] / dt = f(|flow_matrix[i][j]|) - shrink_rate * conductivity[i][j]

public:
    physarum_solver(road_network rn, int vehicle_limit, double vehicle_capacity);
    physarum_solver(road_network rn, int vehicle_limit, double vehicle_capacity, double initial_flux, double initial_conductivity, double shrink_rate, double dt);
    void solve();
};
