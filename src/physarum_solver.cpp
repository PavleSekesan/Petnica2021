#include "physarum_solver.h"
#include "math.h"
#include <iostream>
physarum_solver::physarum_solver(road_network rn, int vehicle_limit, double vehicle_capacity, double initial_flux, double initial_conductivity, double shrink_rate)
 : solver(rn, vehicle_limit, vehicle_capacity)
{
    this->initial_flux = initial_flux;
    this->initial_conductivity = initial_conductivity;
    this->shrink_rate = shrink_rate;

    flow_matrix = std::vector<std::vector<double>>(rn.size(), std::vector<double>(rn.size()));
    conductivity_matrix = std::vector<std::vector<double>>(rn.size(), std::vector<double>(rn.size(), initial_conductivity));
    node_pressures = std::vector<double>(rn.size());
    update_pressures();
    update_flow();
}

void physarum_solver::update_pressures()
{
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
                double increasing_function = current_flow / (1 + current_flow);
                conductivity_matrix[i][j] += increasing_function - shrink_rate * conductivity_matrix[i][j];
            }
        }
    }
}

void physarum_solver::solve()
{
    int iteration_cnt = 100;

    std::cout << "beginning: \n";
    std::cout << "conductivity: \n";
    for(int i = 0; i < rn.size(); i++)
        {
            for(int j = 0; j < rn.size(); j++)
            {
                std::cout << conductivity_matrix[i][j] << " ";
            }
            std::cout << std::endl;
        }
    std::cout << "flow: \n";
    for(int i = 0; i < rn.size(); i++)
        {
            for(int j = 0; j < rn.size(); j++)
            {
                std::cout << flow_matrix[i][j] << " ";
            }
            std::cout << std::endl;
        }

    for(int iteration = 0; iteration < iteration_cnt; iteration++)
    {
        update_flow();
        std::cout << "iteration: " << iteration << std::endl;
        for(int i = 0; i < rn.size(); i++) std::cout << node_pressures[i] << " ";
        std::cout << std::endl;
        for(int i = 0; i < rn.size(); i++)
        {
            for(int j = 0; j < rn.size(); j++)
            {
                std::cout << flow_matrix[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        update_conductivities();
        update_pressures();
    }
}

