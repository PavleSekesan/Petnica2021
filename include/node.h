#pragma once

class node
{
public:
    double x, y, demand;
    node(double x, double y, double demand);
    double distance(node p2);
};
