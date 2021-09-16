#include "evaluator.h"

evaluator::evaluator()
{
    this->rn = std::vector<std::vector<double>>();
}

evaluator::evaluator(road_network rn)
{
    this->rn = rn;
}

double evaluator::evaluate_construction_cost(std::vector<std::vector<bool>> connected)
{
    double construction_cost = 0.0;
    for (int i = 0; i < connected.size(); i++)
    {
        for (int j = 0; j < connected.size(); j++)
        {
            if (connected[i][j])
            {
                construction_cost += rn.get_distance(i, j);
            }
        }
    }
    return construction_cost;
}

double evaluator::evaluate_fault_tolerance(std::vector<std::vector<bool>> connected)
{
    return 0.0;
}

double evaluator::evaluate_weighted_min_distances_sum(std::vector<std::vector<bool>> connected)
{
    // Floyd Warshall algorithm
    std::vector<std::vector<double>> min_distance(rn.size(), std::vector<double>(rn.size(), INFINITY));
    for (int i = 0; i < rn.size(); i++)
    {
        for (int j = 0; j < rn.size(); j++)
        {
            if (connected[i][j])
                min_distance[i][j] = rn.get_distance(i, j);
        }
    }
    for (int i = 0; i < rn.size(); i++)
    {
        min_distance[i][i] = 0.0;
    }

    for (int k = 0; k < rn.size(); k++)
    {
        for (int i = 0; i < rn.size(); i++)
        {
            for (int j = 0; j < rn.size(); j++)
            {
                double intermediate_distance = min_distance[i][k] + min_distance[k][j];
                min_distance[i][j] = std::min(min_distance[i][j], intermediate_distance);
            }
        }
    }

    double weighted_min_distances_sum = 0;
    for (int i = 0; i < rn.size(); i++)
    {
        for (int j = 0; j < rn.size(); j++)
        {
            double demand_i = rn.get_node(i).demand;
            double demand_j = rn.get_node(j).demand;
            weighted_min_distances_sum += min_distance[i][j];
        }
    }

    return weighted_min_distances_sum;
}