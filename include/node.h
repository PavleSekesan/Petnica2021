#pragma once
#include <iostream>

class node
{
public:
    double x, y, demand;
    node(double x, double y, double demand);
    double distance(node p2);
    friend std::ostream& operator<<(std::ostream& os, const node& n);
};
