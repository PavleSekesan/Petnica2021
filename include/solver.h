#pragma once
#include <iostream>
#include "road_network.h"
#include "evaluator.h"
#include <random>

class solver
{
protected:
    road_network rn;
    int vehicle_limit;
    double vehicle_capacity;
    evaluator evaluator;
    std::ostream* output = &std::cout;
public:
    solver(road_network rn);
    void solve(); // returns the found order of visiting each node
    void set_stream(std::ostream& os);
    friend std::ostream& operator<<(std::ostream& os, const solver& s);
};
