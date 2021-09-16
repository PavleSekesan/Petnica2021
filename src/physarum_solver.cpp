#include "physarum_solver.h"
#include "math.h"
#include "nsga_solver.h"
#include <iostream>
#include <iomanip>

physarum_solver::physarum_solver(road_network rn, int vehicle_limit, double vehicle_capacity)
    : solver(rn, vehicle_limit, vehicle_capacity)
{
    flow_matrix = std::vector<std::vector<double>>(rn.size(), std::vector<double>(rn.size()));
    conductivity_matrix = std::vector<std::vector<double>>(rn.size(), std::vector<double>(rn.size(), initial_conductivity));
    node_pressures = std::vector<double>(rn.size());
    flux_vector = std::vector<double>(rn.size());
    initialize_flux();
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

/*
void physarum_solver::update_pressures_exact()
{
    // Pressure system of equations Ax = b
    // A = coefficient_matrix
    // x = solution
    // b = flux_vector

    // Create A (by setting node_pressures[V - 1] = 0, a rectangular matrix is obtained)
    Eigen::MatrixXd coefficient_matrix(rn.size(), rn.size() - 1);
    for (int i = 0; i < rn.size(); i++)
    {
        double current_sum = 0;
        for (int j = 0; j < rn.size() - 1; j++)
        {
            if (j != i)
            {
                double curr_coeff = -conductivity_matrix[i][j] / rn.get_distance(i, j);
                coefficient_matrix(i, j) = curr_coeff;
                current_sum += curr_coeff;
            }
        }
        current_sum += -conductivity_matrix[i][rn.size() - 1] / rn.get_distance(i, rn.size() - 1);
        if (i != rn.size() - 1)
            coefficient_matrix(i, i) = -current_sum;
    }

    // Create b = [initial_flux, 0, 0, 0, 0, ..., 0, -initial_flux] (SHOULD BE MODIFIED FOR VRP)
    Eigen::VectorXd flux_vector(rn.size());
    flux_vector(0) = initial_flux;
    for (int i = 1; i < rn.size() - 2; i++)
        flux_vector(i) = 0;
    flux_vector(rn.size() - 1) = -initial_flux;

    // Solve system using QR decomposition and copy result to node_pressures
    Eigen::VectorXd solution = coefficient_matrix.colPivHouseholderQr().solve(flux_vector);
    Eigen::VectorXd::Map(&node_pressures[0], solution.size()) = solution;
}
*/

void physarum_solver::choose_source_sink_pair()
{
    current_sink_node = rand() % rn.size();
    double cumulative_population = 0.0;
    for (int i = 0; i < rn.size(); i++)
    {
        cumulative_population += rn.get_node(i).demand;
    }

    // Sink
    double rand_number = (double)rand() / RAND_MAX;
    double prefix = 0.0;
    for (int i = 0; i < rn.size(); i++)
    {
        prefix += rn.get_node(i).demand / cumulative_population;
        if (prefix >= rand_number)
        {
            current_sink_node = i;
            break;
        }
    }

    cumulative_population -= rn.get_node(current_sink_node).demand;
    // Source
    rand_number = (double)rand() / RAND_MAX;
    prefix = 0.0;
    for (int i = 0; i < rn.size(); i++)
    {
        if (i != current_sink_node)
        {
            prefix += rn.get_node(i).demand / cumulative_population;
            if (prefix >= rand_number)
            {
                current_source_node = i;
                break;
            }
        }
    }
    /*double cumulative_distance_to_sink = 0.0;
    for (int i = 0; i < rn.size(); i++)
    {
        if (i != current_sink_node)
        {
            cumulative_distance_to_sink += rn.get_distance(current_sink_node, i);
        }
    }
    double rand_number2 = (double)rand() / RAND_MAX;
    double prefix2 = 0.0;
    for (int i = 0; i < rn.size(); i++)
    {
        if (i != current_sink_node)
        {
            prefix2 += rn.get_distance(current_sink_node, i) / cumulative_distance_to_sink;
            if (prefix2 >= rand_number2)
            {
                current_source_node = i;
                break;
            }
        }
    }*/
}

void physarum_solver::update_pressures()
{
    // node_pressures[V - 1] = 0
    for (int i = 0; i < rn.size() - 1; i++)
    {
        double numerator = flux_vector[i];
        double denominator = 0;
        for (int j = 0; j < rn.size(); j++)
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

void physarum_solver::initialize_flux()
{
    if (!rn.is_planar())
    {
        double node_demand_sum = 0;
        for (int i = 1; i < rn.size(); i++)
        {
            node_demand_sum += rn.get_node(i).demand;
        }

        // normalize node demands to determine flux vector
        flux_vector[0] = initial_flux;
        for (int i = 1; i < rn.size(); i++)
        {
            // FIX THIS
            flux_vector[i] = -initial_flux * (1 - rn.get_node(i).demand / node_demand_sum);
        }

        /*std::cout << "Flux vector:\n";
        for (int i = 0; i < rn.size(); i++)
        {
            std::cout << flux_vector[i] << " ";
        }
        std::cout << "\n";*/
    }
    else
    {
        for (int i = 0; i < rn.size(); i++)
            flux_vector[i] = 0;
        flux_vector[current_source_node] = initial_flux;
        flux_vector[current_sink_node] = -initial_flux;
    }
}

void physarum_solver::update_flow()
{
    for (int i = 0; i < rn.size(); i++)
    {
        for (int j = 0; j < rn.size(); j++)
        {
            if (j != i)
            {
                double pressure_difference = node_pressures[i] - node_pressures[j];
                flow_matrix[i][j] = conductivity_matrix[i][j] / rn.get_distance(i, j) * pressure_difference;
                //if (abs(flow_matrix[i][j]) < epsilon)
                //    flow_matrix[i][j] = 0.0;
                if (isnan(flow_matrix[i][j]))
                {
                    std::cout << rn.get_distance(i, j) << " ";
                    std::cout << pressure_difference << " ";
                    std::cout << "AAAAAAA";
                    while (true) {}
                }
            }
        }
    }
}

void physarum_solver::update_conductivities()
{
    is_converged = true;
    for (int i = 0; i < rn.size(); i++)
    {
        for (int j = 0; j < rn.size(); j++)
        {
            if (j != i)
            {
                double current_flow = abs(flow_matrix[i][j]);
                double power = pow(current_flow, gamma);
                double increasing_function = power / (1.0 + power);
                double change = (increasing_function - shrink_rate * conductivity_matrix[i][j]) * dt;
                conductivity_matrix[i][j] += change;
                if (abs(change) > epsilon) is_converged = false;
            }
        }
    }
}

std::vector<int> physarum_solver::solve()
{
    //*output << iteration_cnt << std::endl;
    //for (int iteration = 0; iteration < iteration_cnt; iteration++)
    int iteration = 0;
    while(iteration < iteration_cnt)
    {
        choose_source_sink_pair();
        initialize_flux();
        update_pressures();
        update_flow();
        update_conductivities();
        iteration++;
    }

    *output << std::setprecision(2) << std::fixed;
    *output << *this << std::endl;

    // Determine path by choosing edges with largest flow
    std::vector<int> path;
    int current_node = 0;
    path.push_back(current_node);
    /*while (current_node != rn.size() - 1)
    {
        double max_flow = -INFINITY;
        int node_max = -1;
        for (int i = 0; i < rn.size(); i++)
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
    }*/

    std::vector<std::vector<bool>> connected(rn.size(), std::vector<bool>(rn.size(), false));
    for (int i = 0; i < rn.size(); i++)
    {
        for (int j = 0; j < rn.size(); j++)
        {
            if (conductivity_matrix[i][j] >= 0.01)
                connected[i][j] = true;
        }
    }
    *output << evaluator.evaluate_construction_cost(connected) << std::endl;
    *output << evaluator.evaluate_weighted_min_distances_sum(connected) << std::endl;
    return path;
}

std::ostream& operator<<(std::ostream& os, const physarum_solver& ps)
{
    bool print_flow = false;
    bool print_conductivities = true;
    bool print_pressures = false;
    int print_size_limit = 1000000;
    auto n = ps.rn.size();

    if (n <= print_size_limit)
    {
        // Print flow
        if (print_flow)
        {
            //os << "Flow: \n";
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    os << ps.flow_matrix[i][j] << " ";
                }
                os << std::endl;
            }
        }

        // Print conductivities
        if (print_conductivities)
        {
            //os << "Conductivities: \n";
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    os << ps.conductivity_matrix[i][j] << " ";
                }
                os << std::endl;
            }
        }

        // Print pressures
        if (print_pressures)
        {
            os << "Pressures: \n";
            for (int i = 0; i < n; i++)
                os << ps.node_pressures[i] << " ";
        }
    }
    else
    {
        os << "Solver too large to print";
    }
    return os;
}

