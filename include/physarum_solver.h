#pragma once
#include <iostream>
#include "road_network.h"
#include "solver.h"

class physarum_solver : public solver
{
private:
    double initial_flux = 7.0; // flow rate through the system
    double initial_conductivity = 1.0; // starting "width" of tubes
    double shrink_rate = 0.6; // how fast the tubes shrink in response to flow through a tube
    double dt = 0.01; // time step
    double gamma = 1.8;
    int iteration_cnt = 50000;
    double epsilon = 0.01;
    bool is_converged = false;
    int current_source_node = 0;
    int current_sink_node = rn.size() - 1;
    std::vector<double> flux_vector; // how much flux is generated from each node
    std::vector<double> node_pressures; // pressure for each node in system
    std::vector<std::vector<double>> conductivity_matrix; // conductivity_matrix[i][j] = conductivity between node i and node j
    std::vector<std::vector<double>> flow_matrix; // flow_matrix[i][j] = flow between node i and node j

    void choose_source_sink_pair();
    void update_pressures(); // determines node pressures approximately by plugging in pressures from previous iteration (O(V^2))
    void initialize_flux();
    void update_flow(); // determines flow through tube, defined as conductivity_matrix[i][j] / edge_weigth[i][j] * (pressure[i] - pressure[j])
    void update_conductivities(); // determines conductivities by numerically solving the differential equation d conductivity[i][j] / dt = f(|flow_matrix[i][j]|) - shrink_rate * conductivity[i][j]

public:
    physarum_solver(road_network rn);
    physarum_solver(road_network rn, double initial_flux, double initial_conductivity, double shrink_rate, double dt, double gamma, int iteration_cnt);
    void solve();
    friend std::ostream& operator<<(std::ostream& os, const physarum_solver& ps);
};
