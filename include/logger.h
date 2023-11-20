#ifndef LOGGER_H
#define LOGGER_H

#include "simParameters.h"
#include "udp_sock.h"




 // Structure to log target
typedef struct{
    double x, y, z;
}TARGET_LOG;


// Structure to log solution
typedef struct SOLUTIONS_LOG{
    float rank;
    int number_of_sensors;
    SOLUTIONS_LOG() : number_of_sensors(g_n_sensors_total){}
}SOLUTIONS_LOG;

//UDP Parameters
extern int logger_port;
// RadioMsg target_msg;
extern unsigned short int TARGET_MSG_TYPE;    
extern unsigned short int TARGET_EST_MSG_TYPE;    
extern unsigned short int SOLUTION_RANK_MSG_TYPE;    
extern unsigned short int SOLUTION_COORDS_MSG_TYPE;    
extern unsigned short int SOLUTION_TYPES_MSG_TYPE;    

// For logging with UDP and msgrouter
extern int system_id;
extern int system_subid;
extern int InputPort;

// Functions for logging
UDPSocket openLogSocket();


void logTarget(const std::vector<Target> &target, UDPSocket &sock);

void logSolution(std::vector<Solution> solvec, UDPSocket sock);


#endif