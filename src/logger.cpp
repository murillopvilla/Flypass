#include "../include/solution.h"
#include "../include/logger.h"
#include<unistd.h>


//UDP Parameters
int logger_port = 10101;
// RadioMsg target_msg;
unsigned short int TARGET_MSG_TYPE = 233;    
unsigned short int TARGET_EST_MSG_TYPE = 235;    
unsigned short int SOLUTION_RANK_MSG_TYPE = 234;    
unsigned short int SOLUTION_COORDS_MSG_TYPE = 236;    
unsigned short int SOLUTION_TYPES_MSG_TYPE = 237;    

// For logging with UDP and msgrouter
int system_id = 1;
int system_subid = 10;
int InputPort = 10059;



UDPSocket openLogSocket(){
    // Communication sockets
    UDPSocket sock(InputPort, system_subid, system_id);
    sock.AddLog(logger_port);
    return sock;
};


void logTarget(const std::vector<Target> &target, UDPSocket &sock){

    for (auto &t : target){
        TARGET_LOG target_log;  
        for (auto& point : t.tags){ 
            target_log.x = point.x;
            target_log.y = point.y;
            target_log.z = point.z;
            sock.SendLog(TARGET_MSG_TYPE, &target_log, sizeof(TARGET_LOG));
          
          }  usleep(1000);
        }
};

void logSolution(std::vector<Solution> solvec, UDPSocket sock){
    // LOG SOLUTIONS
    SOLUTIONS_LOG solution_log;
    double sensor_coords[g_n_sensors_total *3]; // to log sensor coordinates
    int sensor_types[g_n_sensors_total]; // to log sensor types
    for (auto& solution : solvec){
        solution_log.rank = solution.rank;
        int sensor_index = 0;
        int sensor_type_index = 0;
        for (auto& sensor: solution.sensors){
            sensor_coords[sensor_index] = sensor.pos.x;
            sensor_coords[sensor_index+1] = sensor.pos.y;
            sensor_coords[sensor_index+2] = sensor.pos.z;
            sensor_types[sensor_type_index] = sensor.type;
            sensor_index +=3;
            sensor_type_index ++;
        }
        sock.SendLog(SOLUTION_RANK_MSG_TYPE, &solution_log, sizeof(SOLUTIONS_LOG));
        usleep(1000);
        sock.SendLog(SOLUTION_COORDS_MSG_TYPE, &sensor_coords, sizeof(sensor_coords));
        usleep(1000);
        sock.SendLog(SOLUTION_TYPES_MSG_TYPE, &sensor_types, sizeof(sensor_types));
        usleep(1000);
    }
};

