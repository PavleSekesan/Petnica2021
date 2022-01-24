#include "evaluator.h"

evaluator::evaluator()
{
    this->rn = std::vector<std::vector<double>>();
}

evaluator::evaluator(road_network rn)
{
    this->rn = rn;
    // Max cost
    max_cost = 0.0;
    for (int i = 0; i < rn.size(); i++)
    {
        for (int j = 0; j < rn.size(); j++)
        {
            max_cost += rn.get_distance(i, j);
        }
    }

    int n = rn.size();
    std::vector<std::vector<bool>> mst(n, std::vector<bool>(n));

    struct Edge {
        double w = INFINITY, to = -1;
    };

    double mst_weight = 0;
    std::vector<bool> selected(n, false);
    std::vector<Edge> min_e(n);
    min_e[0].w = 0;

    for (int i = 0; i < n; ++i) {
        int v = -1;
        for (int j = 0; j < n; ++j) {
            if (!selected[j] && (v == -1 || min_e[j].w < min_e[v].w))
                v = j;
        }

        selected[v] = true;
        mst_weight += min_e[v].w;

        if (min_e[v].to != -1)
        {
            mst[v][min_e[v].to] = true;
            mst[min_e[v].to][v] = true;
        }

        for (int to = 0; to < n; ++to) {
            if (rn.get_distance(v, to) < min_e[to].w)
            {
                min_e[to].w = rn.get_distance(v, to);
                min_e[to].to = v;
            }
        }
    }

    // Print MST
    //for (int i = 0; i < n; i++)
    //{
    //    for (int j = 0; j < n; j++)
    //    {
    //        std::cout << mst[i][j] << " ";
    //    }
    //    std::cout << std::endl;
    //}

    max_fault_tolerance = mst_weight;

    max_weighted_min_distances_sum = evaluate_weighted_min_distances_sum(mst, false);
    
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
    return construction_cost / max_cost;
}

std::vector<std::vector<double>> evaluator::floyd_warshall(std::vector<std::vector<bool>> connected)
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

    return min_distance;
}


void dfs(int v, std::vector<std::vector<int>>& adj, std::vector<bool>& visited, std::vector<int>& tin, std::vector<int>& low, int& timer, std::vector<std::pair<int, int>>& bridges, int p = -1)
{
    visited[v] = true;
    tin[v] = low[v] = timer++;
    for (int to : adj[v]) {
        if (to == p) continue;
        if (visited[to]) {
            low[v] = std::min(low[v], tin[to]);
        }
        else {
            dfs(to, adj, visited, tin, low, timer, bridges, v);
            low[v] = std::min(low[v], low[to]);
            if (low[to] > tin[v])
                bridges.emplace_back(v, to);
        }
    }
}

double evaluator::evaluate_fault_tolerance(std::vector<std::vector<bool>> connected)
{
    std::vector<std::vector<int>> adj(connected.size());
    for (int i = 0; i < connected.size(); i++)
    {
        for (int j = i + 1; j < connected.size(); j++)
        {
            if (connected[i][j])
            {
                adj[i].push_back(j);
                adj[j].push_back(i);
            }
        }
    }
    
    std::vector<std::pair<int, int>> bridges;
    std::vector<bool> visited;
    std::vector<int> tin, low;
    int timer;

    timer = 0;
    visited.assign(connected.size(), false);
    tin.assign(connected.size(), -1);
    low.assign(connected.size(), -1);
    for (int i = 0; i < connected.size(); ++i) {
        if (!visited[i])
            dfs(i, adj, visited, tin, low, timer, bridges);
    }

    double fault_tolerance = 0.0;
    for (auto bridge : bridges)
    {
        fault_tolerance += rn.get_distance(bridge.first, bridge.second);
    }

    return fault_tolerance / max_fault_tolerance;
}

double evaluator::evaluate_weighted_min_distances_sum(std::vector<std::vector<bool>> connected, bool normalized)
{
    double max_demand = -INFINITY;
    for (int i = 0; i < rn.size(); i++)
    {
        max_demand = std::max(max_demand, rn.get_node(i).demand);
    }

    std::vector<std::vector<double>> min_distance = floyd_warshall(connected);

    double weighted_min_distances_sum = 0;
    for (int i = 0; i < rn.size(); i++)
    {
        for (int j = 0; j < rn.size(); j++)
        {
            double demand_i = rn.get_node(i).demand / max_demand;
            double demand_j = rn.get_node(j).demand / max_demand;
            weighted_min_distances_sum += min_distance[i][j];
        }
    }

    if (normalized)
        return weighted_min_distances_sum / max_weighted_min_distances_sum;
    else
        return weighted_min_distances_sum;
}