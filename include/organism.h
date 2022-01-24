#pragma once
#include <vector>
#include "random_gen.h"
class organism
{
private:
	int genome_size;
public:
	organism();
	organism(int genome_size, bool random = false);
	std::vector<std::vector<bool>> genome;
	int rank;
	double crowding_distance;
	std::vector<double> objective_evaluation;
	void mutate(double mutation_rate);
	organism crossover(organism parent);
	bool dominates(organism o);
	bool crowded_comparison(organism o);
};

