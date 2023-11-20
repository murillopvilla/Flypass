#ifndef RUNSIM_H
#define RUNSIM_H

#include <iostream>
#include <stdio.h>

#include "Ekalman.h"
#include "GA.h"



void runSimStatic(const std::vector<Target> &target, std::vector<Solution> &solvec);

void runSimStaticRandom(const std::vector<Target>  &target, std::vector<Solution> &solvec);


#endif