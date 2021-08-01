#pragma once
#include "road_network.h"

class solver
{
protected:
    road_network rn;
    int vehicle_limit;
    double vehicle_capacity;
public:
    solver(road_network rn, int vehicle_limit, double vehicle_capacity);
    void solve();
};
