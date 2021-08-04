#pragma once
#include <iostream>
#include "road_network.h"

class solver
{
protected:
    road_network rn;
    int vehicle_limit;
    double vehicle_capacity;
    std::ostream* output = &std::cout;
public:
    solver(road_network rn, int vehicle_limit, double vehicle_capacity);
    std::vector<int> solve(); // returns the found order of visiting each node
    void set_stream(std::ostream& os);
    friend std::ostream& operator<<(std::ostream& os, const solver& s);
};
