

#include "../include/runSim.h"

void runSimStatic(const std::vector<Target>  &target, std::vector<Solution> &solvec){
// Generate initial random solution vector
    /*
    Each solution is a struct that contains a vector of sensors
    Each sensor is a point struct containing the x, y and z coordinates
    Initially we will generate a vector of random solutions.    */
   
    std::cout << "Start SIM" << std::endl;


    // for testing purposes
    Solution stest;

    sensor s1;
    sensor s2;
    sensor s3;
    sensor s4;

    s1.type = RANGE_SENSOR;
    s2.type = RANGE_SENSOR;
    s3.type = RANGE_SENSOR;
    s4.type = RANGE_SENSOR;


    point p1(2,1);
    point p2(2,0);
    point p3(0,1);
    point p4(0,1);


    s1.pos = p1;
    s2.pos = p2;
    s3.pos = p3;
    s4.pos = p4;

    stest.sensors.push_back(s1);
    stest.sensors.push_back(s2);
    stest.sensors.push_back(s3);
    stest.sensors.push_back(s4);

    stest.fitness(target);




    randomSolInit(solvec, g_pop_size);
    for (auto &s : solvec) s.fitness(target);

    // ------------------- GA LOOP ----------------- //
    for (int i = 0; i < g_ga_iter; i++)
    {       

         std::sort(solvec.begin(), solvec.end(), [](const auto &lhs, const auto &rhs)
               { return lhs.rank < rhs.rank; });
        // std::sort(solvec.begin(), solvec.end(), [](const auto &lhs, const auto &rhs)
        //     { return lhs.rank > rhs.rank; });
        // ------ Tournament
        // Select random individuals for the tournament
        std::vector<Solution> constestants;
        constestants.reserve(g_alpha_p);

        for (int i = 0; i < g_alpha_p; i++)
        {
            Solution sample = randomSample(solvec); // sample random solution in the solutions vector
            constestants.push_back(sample); // add sample solution to the constestants vector
        }       

        // Make sure number of contestants is even for Cross over
        if (constestants.size() % 2 != 0)
            constestants.pop_back();       
        // Evaluate the constestants
        for (auto &s : constestants)
        {
            s.fitness(target);
        }       
        // Sort the constestants by rank
         std::sort(constestants.begin(), constestants.end(), [](const auto &lhs, const auto &rhs)
                  { return lhs.rank < rhs.rank; });

        // std::sort(constestants.begin(), constestants.end(), [](const auto &lhs, const auto &rhs)
        // { return lhs.rank > rhs.rank; });
        // ------ Cross Over
        int num_parents = constestants.size(); // Size of the parents vector is always even

        std::vector<Solution> children;
        children.reserve(num_parents);
        // Initialize  all sensors of all children with any value
        randomSolInit(children, num_parents);        
        CrossOver(constestants, children); 
        // Calculate the rank of the children
        for (auto &c : children) c.fitness(target);
        // Reintegrate the children in the solution vector
        std::copy(children.begin(), children.end(), solvec.end() - children.size());
     // ------ Mutation
        Mutate(solvec, children.size(),target);       
    }

}


void runSimStaticRandom(const std::vector<Target>  &target, std::vector<Solution> &solvec){
// Generate initial random solution vector
    /*
    Each solution is a struct that contains a vector of sensors
    Each sensor is a point struct containing the x, y and z coordinates
    Initially we will generate a vector of random solutions.    */
   
    std::cout << "Start SIM" << std::endl;
    randomSolInit(solvec, 1);

    for (auto &s : solvec) s.fitness(target);

    

    // ------------------- GA LOOP ----------------- //
    for (int i = 0; i < g_ga_iter; i++)
    {       

         std::sort(solvec.begin(), solvec.end(), [](const auto &lhs, const auto &rhs)
               { return lhs.rank < rhs.rank; });
        // std::sort(solvec.begin(), solvec.end(), [](const auto &lhs, const auto &rhs)
        //     { return lhs.rank > rhs.rank; });
        // ------ Tournament
        // Select random individuals for the tournament
        std::vector<Solution> constestants;
        constestants.reserve(g_alpha_p);

        for (int i = 0; i < g_alpha_p; i++)
        {
            Solution sample = randomSample(solvec); // sample random solution in the solutions vector
            constestants.push_back(sample); // add sample solution to the constestants vector
        }       

        // Make sure number of contestants is even for Cross over
        if (constestants.size() % 2 != 0)
            constestants.pop_back();       
        // Evaluate the constestants
        for (auto &s : constestants)
        {
            s.fitness(target);
        }       
        // Sort the constestants by rank
         std::sort(constestants.begin(), constestants.end(), [](const auto &lhs, const auto &rhs)
                  { return lhs.rank < rhs.rank; });

        // std::sort(constestants.begin(), constestants.end(), [](const auto &lhs, const auto &rhs)
        // { return lhs.rank > rhs.rank; });
        // ------ Cross Over
        int num_parents = constestants.size(); // Size of the parents vector is always even

        std::vector<Solution> children;
        children.reserve(num_parents);
        // Initialize  all sensors of all children with any value
        randomSolInit(children, num_parents);        
        CrossOver(constestants, children); 
        // Calculate the rank of the children
        for (auto &c : children) c.fitness(target);
        // Reintegrate the children in the solution vector
        std::copy(children.begin(), children.end(), solvec.end() - children.size());
     // ------ Mutation
        Mutate(solvec, children.size(),target);       
    }

}