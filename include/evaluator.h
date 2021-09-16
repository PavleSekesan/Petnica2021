#pragma once
#include <vector>
#include "road_network.h"
class evaluator
{
private:
    road_network rn;
public:
    evaluator();
    evaluator(road_network rn);
    double evaluate_construction_cost(std::vector<std::vector<bool>> connected);
    double evaluate_fault_tolerance(std::vector<std::vector<bool>> connected);
    double evaluate_weighted_min_distances_sum(std::vector<std::vector<bool>> connected);
    
};

