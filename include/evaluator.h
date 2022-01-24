#pragma once
#include <vector>
#include "road_network.h"
class evaluator
{
private:
    road_network rn;
    double max_cost;
    double max_fault_tolerance;
    double max_weighted_min_distances_sum;
    std::vector<std::vector<double>> floyd_warshall(std::vector<std::vector<bool>> connected);
public:
    evaluator();
    evaluator(road_network rn);
    double evaluate_construction_cost(std::vector<std::vector<bool>> connected);
    double evaluate_fault_tolerance(std::vector<std::vector<bool>> connected);
    double evaluate_weighted_min_distances_sum(std::vector<std::vector<bool>> connected, bool normalized = true);
    
};

