#include "solver.h"

solver::solver(road_network rn)
{
    this->rn = rn;
    this->evaluator = evaluator::evaluator(rn);
}

void solver::set_stream(std::ostream& os)
{
    output = &os;
}