#include "solver.h"

solver::solver(road_network rn, int vehicle_limit, double vehicle_capacity)
{
    this->rn = rn;
    this->vehicle_limit = vehicle_limit;
    this->vehicle_capacity = vehicle_capacity;
}

void solver::set_stream(std::ostream& os)
{
    output = &os;
}