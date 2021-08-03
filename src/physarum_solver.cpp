#include "physarum_solver.h"
#include "math.h"
#include <iostream>
#include <../Eigen/QR>

physarum_solver::physarum_solver(road_network rn, int vehicle_limit, double vehicle_capacity)
 : solver(rn, vehicle_limit, vehicle_capacity)
{
    flow_matrix = std::vector<std::vector<double>>(rn.size(), std::vector<double>(rn.size()));
    conductivity_matrix = std::vector<std::vector<double>>(rn.size(), std::vector<double>(rn.size(), initial_conductivity));
    node_pressures = std::vector<double>(rn.size());
}

physarum_solver::physarum_solver(road_network rn, int vehicle_limit, double vehicle_capacity, double initial_flux, double initial_conductivity, double shrink_rate, double dt)
 : solver(rn, vehicle_limit, vehicle_capacity)
{
    this->initial_flux = initial_flux;
    this->initial_conductivity = initial_conductivity;
    this->shrink_rate = shrink_rate;
    this->dt = dt;
    physarum_solver(rn, vehicle_limit, vehicle_capacity);
}

void physarum_solver::update_pressures_exact()
{
    // Pressure system of equations Ax = b
    // A = coefficient_matrix
    // x = solution
    // b = flux_vector

    // Create A (by setting node_pressures[V - 1] = 0, a rectangular matrix is obtained)
    Eigen::MatrixXd coefficient_matrix(rn.size(), rn.size() - 1);
    for(int i = 0; i < rn.size(); i++)
    {
        double current_sum = 0;
        for(int j = 0; j < rn.size() - 1; j++)
        {
            if (j != i)
            {
                double curr_coeff = - conductivity_matrix[i][j] / rn.get_distance(i, j);
                coefficient_matrix(i, j) = curr_coeff;
                current_sum += curr_coeff;
            }
        }
        current_sum += - conductivity_matrix[i][rn.size() - 1] / rn.get_distance(i, rn.size() - 1);
        if (i != rn.size() - 1)
            coefficient_matrix(i, i) = -current_sum;
    }

    // Create b = [initial_flux, 0, 0, 0, 0, ..., 0, -initial_flux]
    Eigen::VectorXd flux_vector(rn.size());
    flux_vector(0) = initial_flux;
    for(int i = 1; i < rn.size() - 2; i++)
        flux_vector(i) = 0;
    flux_vector(rn.size() - 1) = -initial_flux;

    // Solve system using QR decomposition and copy result to node_pressures
    Eigen::VectorXd solution = coefficient_matrix.colPivHouseholderQr().solve(flux_vector);
    Eigen::VectorXd::Map(&node_pressures[0], solution.size()) = solution;
}

void physarum_solver::update_pressures()
{
    // node_pressures[V - 1] = 0
    for(int i = 0; i < rn.size() - 1; i++)
    {
        double numerator = 0;
        if (i == 0) numerator = initial_flux;
        double denominator = 0;
        for(int j = 0; j < rn.size(); j++)
        {
            if (j != i)
            {
                double coeff = conductivity_matrix[i][j] / rn.get_distance(i, j);
                numerator += coeff * node_pressures[j];
                denominator += coeff;
            }
        }
        node_pressures[i] = numerator / denominator;
    }
    return;
}

void physarum_solver::update_flow()
{
    for(int i = 0; i < rn.size(); i++)
    {
        for(int j = 0; j < rn.size(); j++)
        {
            if (j != i)
            {
                double pressure_difference = node_pressures[i] - node_pressures[j];
                flow_matrix[i][j] = conductivity_matrix[i][j] / rn.get_distance(i, j) * pressure_difference;
            }
        }
    }
}

void physarum_solver::update_conductivities()
{
    for(int i = 0; i < rn.size(); i++)
    {
        for(int j = 0; j < rn.size(); j++)
        {
            if (j != i)
            {
                double current_flow = abs(flow_matrix[i][j]);
                double increasing_function = current_flow;
                conductivity_matrix[i][j] += (increasing_function - shrink_rate * conductivity_matrix[i][j]) * dt;
            }
        }
    }
}

void physarum_solver::solve()
{
    int iteration_cnt = 100;

    for(int iteration = 0; iteration < iteration_cnt; iteration++)
    {
        update_pressures();
        update_flow();
        update_conductivities();
    }

    std::vector<int> path;
    int current_node = 0;
    path.push_back(current_node);
    while(current_node != rn.size() - 1)
    {
        double max_flow = -INFINITY;
        int node_max = -1;
        for(int i = 0; i < rn.size(); i++)
        {
            if (i != current_node)
            {
                double curr_weight = flow_matrix[current_node][i];
                if (curr_weight > max_flow)
                {
                    max_flow = curr_weight;
                    node_max = i;
                }
            }
        }
        path.push_back(node_max);
        current_node = node_max;
    }

    double path_length = 0;
    for(int i = 0; i < path.size(); i++)
    {
        std::cout << path[i] << " ";
    }
    std::cout << std::endl;
    for(int i = 0; i < path.size() - 1; i++)
    {
        path_length += rn.get_distance(path[i], path[i + 1]);
    }
    std::cout << path_length << std::endl;
}

