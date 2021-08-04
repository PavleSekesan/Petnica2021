#include "node.h"
#include <math.h>

node::node(double x, double y, double demand)
{
    this->x = x;
    this->y = y;
    this->demand = demand;
}

double node::distance(node p2)
{
    double dx = x - p2.x;
    double dy = y - p2.y;

    return std::sqrt(dx * dx + dy * dy);
}

std::ostream& operator<<(std::ostream& os, const node& n)
{
    os << "X: " << n.x << " Y: " << n.y;
    return os;
}
