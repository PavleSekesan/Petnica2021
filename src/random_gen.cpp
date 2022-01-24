#include "random_gen.h"

std::mt19937 random_gen::rng;

double random_gen::uniform_real()
{
    return std::uniform_real_distribution<double>(0, 1)(rng);
}

int random_gen::uniform_int(int max)
{
    return std::uniform_int_distribution<int>(0, max - 1)(rng);
}
