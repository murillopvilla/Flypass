#ifndef SIM_PARAMETERS_H
#define SIM_PARAMETERS_H


#include <string>
#include <eigen3/Eigen/Dense>

#include "xmlParser.h"

// SENSOR TYPES


#define RANGE_SENSOR 0
#define ANGLE_SENSOR 1
#define TOA_SENSOR 2
#define TDOA_SENSOR 3



// -------- GLOBAL VARIABLES

// Map variables
extern float g_xgridi;
extern float g_ygridi;
extern float g_xgridf;
extern float g_ygridf;

extern float g_xcenter;
extern float g_ycenter;


extern int g_z_offset;


extern int g_motion;
// Maneuver

extern std::string g_maneuver;

// Range Measurement parameters
extern float g_eta_r; // range multiplication factor
extern float g_mi_r; // noise mean (bias)
extern float g_sigma_r; // constant measurement variance




// Angle Measurement parameters
extern float g_eta_a; // angle multiplication factor
extern float g_mi_a; // angle mean (bias)
extern float g_sigma_a; // constant measurement variance





// TOA Measurement parameters
extern float g_eta_toa; // angle multiplication factor
extern float g_mi_toa; // angle mean (bias)
extern float g_sigma_toa; // constant measurement variance



// TDOA Measurement parameters
extern float g_eta_tdoa; // angle multiplication factor
extern float g_mi_tdoa; // angle mean (bias)
extern float g_sigma_tdoa; // constant measurement variance




extern float g_c; // sound velocity in water

// GA parameters
extern int g_pop_size; // number of individuals in the population
extern int g_ga_iter; // number of iterations for the GA
extern int g_local_search_iter; // number of iterations for the GA
extern int g_local_search_version;

extern std::string g_criteria;

extern int g_weigths_flag;


extern int g_n_sensors_angle;  // number of sensors in each solution
extern int g_n_sensors_range;  // number of sensors in each solution
extern int g_n_sensors_toa;  // number of sensors in each solution
extern int g_n_sensors_tdoa;  // number of sensors in each solution


extern int g_n_sensors_total;  // number of sensors in each solution



extern int g_alpha_p;    // numer of constestants for tournament
extern int g_alpha_m;    // numer of individuals for mutation



extern Eigen::MatrixXd g_Q_range;


extern Eigen::MatrixXd g_Q_angle;


extern Eigen::MatrixXd g_Q_toa;


extern Eigen::MatrixXd g_Q_tdoa;


extern int g_range_measurement_period;

extern int g_angle_measurement_period;

extern int g_toa_measurement_period;

extern int g_tdoa_measurement_period;

// Propagation model covariances


extern Eigen::MatrixXd g_R;




extern double g_dt; // time step for propagation model


// Initialize the target states and covariances

extern double g_xti, g_yti, g_zti; // initialization on main()



extern double g_vxi;
extern double g_vyi;
extern double g_vzi;

extern double g_cx;
extern double g_cy;
extern double g_cz;



void readParameters();


#endif