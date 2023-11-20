#ifndef GA_H
#define GA_H


#include <stdio.h>
#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <random>
#include <algorithm>
#include <chrono>

#include "simParameters.h"
#include "solution.h"




// --------- Function to sample random elements of vector

// --------- Function to sample random elements of vector

template <typename T>
T randomSample(std::vector<T> const &v)
{    auto it = v.cbegin();
    int random = rand() % v.size();    
    std::advance(it, random);
    return *it;
}

// ---------- GA DECLARATIONS ------------------- //

void randomSolInit(std::vector<Solution> &solvec, int localpopsize);
void CrossOver(std::vector<Solution> &P, std::vector<Solution> &children);
void Mutate(std::vector<Solution> &S, std::size_t c, const std::vector<Target> &target);
Solution LocalSearch(const std::vector<point> &tpoint, Solution &current_solution);
Solution LocalSearchV2(const std::vector<point> &tpoint, Solution &current_solution);






#endif
