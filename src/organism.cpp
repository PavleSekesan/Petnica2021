#include "organism.h"

organism::organism()
{
	genome = std::vector<std::vector<bool>>();
}

organism::organism(int genome_size, bool random)
{
	this->genome_size = genome_size;
	rank = -1;
	crowding_distance = -INFINITY;
	objective_evaluation = std::vector<double>(2);
	double ones = 0;
	genome = std::vector<std::vector<bool>>(genome_size, std::vector<bool>(genome_size));
	if (random)
	{
		for (int i = 0; i < genome_size; i++)
		{
			for (int j = i + 1; j < genome_size; j++)
			{
				if (ones <= 100)
				{
					genome[i][j] = rand() % 2;
					genome[j][i] = genome[i][j];
					ones += genome[i][j];
				}
			}
		}
	}
}

void organism::mutate(double mutation_rate)
{
	double rng = (double)rand() / RAND_MAX;
	if (rng <= mutation_rate)
	{
		for (int i = 0; i < 1; i++)
		{
			int rand_node1 = rand() % genome_size;
			int rand_node2 = rand() % genome_size;
			genome[rand_node1][rand_node2] = !genome[rand_node1][rand_node2];
			genome[rand_node2][rand_node1] = genome[rand_node1][rand_node2];
		}
	}
}

organism organism::crossover(organism parent)
{
	organism child = organism(genome_size);
	for (int i = 0; i < genome_size; i++)
	{
		for (int j = i + 1; j < genome_size; j++)
		{
			bool pass_my_gene = rand() % 2;
			if (pass_my_gene)
			{
				child.genome[i][j] = this->genome[i][j];
			}
			else // pass other parents' gene
			{
				child.genome[i][j] = parent.genome[i][j];
			}
			child.genome[j][i] = child.genome[i][j];
		}
	}

	return child;
}

bool organism::dominates(organism o)
{
	bool one_better = false;
	bool others_same = true;
	for (int i = 0; i < objective_evaluation.size(); i++)
	{
		if (this->objective_evaluation[i] > o.objective_evaluation[i])
			others_same = false;
		if (this->objective_evaluation[i] < o.objective_evaluation[i])
			one_better = true;
	}

	return one_better && others_same;
}

bool organism::crowded_comparison(organism o)
{
	return (this->rank < o.rank) || ((this->rank == o.rank) && (this->crowding_distance > o.crowding_distance));
}
