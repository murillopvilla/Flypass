#include <iostream>
#include <stdio.h>
#include <random>
#include <vector>
#include <string>
#include <ctime>    
#include <string>
#include <chrono>
#include<unistd.h>


//#include "include/Ekalman.h"
#include "include/readManeuver.h"
#include "include/runSim.h"
#include "include/logger.h"

#include "protocol_msg.h"
#include "udp_sock.h"




//const int nsensors = 8;

int main(){
    readParameters();
    // -------- Read target maneuver from file ---------
    std::string path = "Trajectory_files/3000_3000_1000/" + g_maneuver + ".txt";


    const std::vector<Target> target = readManeuverFile2(path);   

    //const Target target = readManeuverFile(path);   


    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Solution> solvec; // Vector to store the solutions
    solvec.reserve(g_pop_size);
    // RUN SIM
    runSimStatic(target, solvec);   
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> duration = end - start;
    std::cout << "Time total: " << duration.count() << "s" << std::endl;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "finished computation at " << std::ctime(&end_time);
    // Sort the solution vector by the rank
    // std::sort(solvec.begin(), solvec.end(), [](const auto &lhs, const auto &rhs)
    //           { return lhs.rank > rhs.rank; });

    std::sort(solvec.begin(), solvec.end(), [](const auto &lhs, const auto &rhs)
              { return lhs.rank > rhs.rank; });
    // Logging
    UDPSocket sock = openLogSocket();
    logTarget(target, sock);
    logSolution(solvec, sock);   

   std::cout << "-------------------------------- END ------------------------------" << std::endl;
    return 0;
}