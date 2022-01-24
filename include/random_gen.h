#pragma once
#include <random>
#include <chrono>
class random_gen
{
public:
	static std::mt19937 rng;
	static double uniform_real();
	static int uniform_int(int max);
};

