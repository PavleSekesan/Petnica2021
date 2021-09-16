#include "nsga_solver.h"

void nsga_solver::generate_new_population()
{
    std::vector<organism> children;
    while (children.size() < population.size())
    {
        organism parent1 = organism(rn.size());
        organism parent2 = organism(rn.size());
        organism o1 = population[rand() % population.size()];
        organism o2 = population[rand() % population.size()];
        if (o1.crowded_comparison(o2))
        {
            parent1 = o1;
        }
        else
        {
            parent1 = o2;
        }
        o1 = population[rand() % population.size()];
        o2 = population[rand() % population.size()];
        if (o1.crowded_comparison(o2))
        {
            parent2 = o1;
        }
        else
        {
            parent2 = o2;
        }
        organism child = parent1.crossover(parent2);
        child.mutate(mutation_rate);
        children.push_back(child);
    }

    for (int i = 0; i < children.size(); i++)
    {
        population.push_back(children[i]);
    }
}

void nsga_solver::evaluate_population()
{
    for (int i = 0; i < population.size(); i++)
    {
        population[i].objective_evaluation[0] = evaluator.evaluate_construction_cost(population[i].genome);
        //population[i].objective_evaluation[1] = evaluator.evaluate_fault_tolerance(population[i].genome);
        population[i].objective_evaluation[1] = evaluator.evaluate_weighted_min_distances_sum(population[i].genome);
    }
}

std::vector<std::vector<organism>> nsga_solver::nondominated_sort()
{
    std::vector<std::vector<int>> fronts(1);
    std::vector<std::vector<int>> dominated_organisms_map(population.size());
    std::vector<int> domination_number_map(population.size());
    for (int o1 = 0; o1 < population.size(); o1++)
    {
        std::vector<int> dominated_organisms;
        int domination_number = 0;
        for (int o2 = 0; o2 < population.size(); o2++)
        {
            bool o1_domination = population[o1].dominates(population[o2]);
            bool o2_domination = population[o2].dominates(population[o1]);
            if (o1_domination)
            {
                dominated_organisms.push_back(o2);
            }
            else if (o2_domination)
            {
                domination_number++;
            }
        }
        if (domination_number == 0)
        {
            population[o1].rank = 0;
            fronts[0].push_back(o1);
        }
        dominated_organisms_map[o1] = dominated_organisms;
        domination_number_map[o1] = domination_number;
    }

    int front_index = 0;
    while (!fronts[front_index].empty())
    {
        std::vector<int> next_front;
        for (int o1 : fronts[front_index])
        {
            for (int o2 : dominated_organisms_map[o1])
            {
                domination_number_map[o2]--;
                if (domination_number_map[o2] == 0)
                {
                    population[o2].rank = front_index + 1;
                    next_front.push_back(o2);
                }
            }
        }
        front_index++;
        fronts.push_back(next_front);
    }

    std::vector<std::vector<organism>> organism_fronts(fronts.size());
    for (int i = 0; i < fronts.size(); i++)
    {
        organism_fronts[i] = std::vector<organism>(fronts[i].size());
        for (int j = 0; j < fronts[i].size(); j++)
        {
            organism_fronts[i][j] = population[fronts[i][j]];
        }
    }
    return organism_fronts;
}

void nsga_solver::crowding_distance_assignment(std::vector<organism>& front)
{
    if (!front.empty())
    {
        for (int i = 0; i < front.size(); i++)
            front[i].crowding_distance = 0;
        for (int objective = 0; objective < front[0].objective_evaluation.size(); objective++)
        {
            std::sort(front.begin(), front.end(), [objective](organism o1, organism o2) { return o1.objective_evaluation[objective] < o2.objective_evaluation[objective]; });
            double normalization = front[front.size() - 1].objective_evaluation[objective] - front[0].objective_evaluation[objective];
            if (normalization <= 0.0001) normalization = 1;
            front[0].crowding_distance = INFINITY;
            front[front.size() - 1].crowding_distance = INFINITY;
            for (int i = 1; i < (int)front.size() - 2; i++)
            {
                front[i].crowding_distance += (front[i + 1].objective_evaluation[objective] - front[i - 1].objective_evaluation[objective]) / normalization;
            }
        }
    }
}



nsga_solver::nsga_solver(road_network rn, int vehicle_limit, double vehicle_capacity)
    : solver(rn, vehicle_limit, vehicle_capacity)
{
}

std::vector<int> nsga_solver::solve()
{
    // Randomly generated population P0
    population = std::vector<organism>(population_size);
    for (int i = 0; i < population_size; i++)
    {
        population[i] = organism(rn.size(), true);
    }
    evaluate_population();
    std::vector<std::vector<organism>> fronts = nondominated_sort();
    for (int i = 0; i < fronts.size(); i++)
    {
        crowding_distance_assignment(fronts[i]);
    }
    generate_new_population();
    evaluate_population();

    for (int generation = 0; generation < generation_count; generation++)
    {
        std::vector<std::vector<organism>> fronts = nondominated_sort();
        std::vector<organism> new_population;
        int front_index = 0;
        while (new_population.size() + fronts[front_index].size() <= population_size)
        {
            crowding_distance_assignment(fronts[front_index]);
            for (int i = 0; i < fronts[front_index].size(); i++)
                new_population.push_back(fronts[front_index][i]);
            front_index++;
        }
        crowding_distance_assignment(fronts[front_index]);
        std::sort(fronts[front_index].begin(), fronts[front_index].end(), [](organism o1, organism o2) { return o1.crowded_comparison(o2); });
        int last_front_cutoff = population_size - new_population.size();
        for (int i = 0; i < last_front_cutoff; i++)
        {
            new_population.push_back(fronts[front_index][i]);
        }
        population = new_population;
        double min_distance = INFINITY;
        int min_ind = -1;
        for (int i = 0; i < population.size(); i++)
        {
            if (population[i].objective_evaluation[0] < min_distance)
            {
                min_distance = population[i].objective_evaluation[0];
                min_ind = i;
            }
            
        }
        std::cout << min_ind << ": " << min_distance << std::endl;
        evaluate_population();
        generate_new_population();
        evaluate_population();
    }

    *output << std::setprecision(2) << std::fixed;
    *output << *this << std::endl;

    return std::vector<int>();
}

std::ostream& operator<<(std::ostream& os, const nsga_solver& ns)
{
    std::cout << "Final organisms:" << std::endl;
    for (int i = 0; i < ns.population.size(); i++)
    {
        for (int j = 0; j < ns.population[i].objective_evaluation.size(); j++)
        {
            std::cout << ns.population[i].objective_evaluation[j] << " ";
        }
        std::cout << std::endl;
    }
    std::vector<std::vector<bool>> connected = ns.population[1].genome;
    for (int i = 0; i < connected.size(); i++)
    {
        for (int j = 0; j < connected.size(); j++)
        {
            os << connected[i][j] << " ";
        }
        os << std::endl;
    }
    return os;
}
