#include "../include/simParameters.h"


// -------- GLOBAL VARIABLES

// -------- GLOBAL VARIABLES

// Map variables
float g_xgridi;
float g_ygridi;
float g_xgridf;
float g_ygridf;

float g_xcenter;
float g_ycenter;

int g_z_offset;

int g_motion;
// Maneuver

std::string g_maneuver;

// Range Measurement parameters
float g_eta_r; // range multiplication factor
float g_mi_r; // noise mean (bias)
float g_sigma_r; // constant measurement variance




// Angle Measurement parameters
float g_eta_a; // angle multiplication factor
float g_mi_a; // angle mean (bias)
float g_sigma_a; // constant measurement variance





// TOA Measurement parameters
float g_eta_toa; // angle multiplication factor
float g_mi_toa; // angle mean (bias)
float g_sigma_toa; // constant measurement variance



// TDOA Measurement parameters
float g_eta_tdoa; // angle multiplication factor
float g_mi_tdoa; // angle mean (bias)
float g_sigma_tdoa; // constant measurement variance




float g_c; // sound velocity in water

// GA parameters
int g_pop_size; // number of individuals in the population
int g_ga_iter; // number of iterations for the GA
int g_local_search_iter; // number of iterations for the GA
int g_local_search_version;

std::string g_criteria;


int g_weigths_flag;


int g_n_sensors_angle;  // number of sensors in each solution
int g_n_sensors_range;  // number of sensors in each solution
int g_n_sensors_toa;  // number of sensors in each solution
int g_n_sensors_tdoa;  // number of sensors in each solution


int g_n_sensors_total;  // number of sensors in each solution



int g_alpha_p;    // numer of constestants for tournament
int g_alpha_m;    // numer of individuals for mutation



Eigen::MatrixXd g_Q_range;


Eigen::MatrixXd g_Q_angle;


Eigen::MatrixXd g_Q_toa;


Eigen::MatrixXd g_Q_tdoa;


int g_range_measurement_period;

int g_angle_measurement_period;

int g_toa_measurement_period;

int g_tdoa_measurement_period;

// Propagation model covariances


Eigen::MatrixXd g_R;




double g_dt; // time step for propagation model


// Initialize the target states and covariances

double g_xti, g_yti, g_zti; // initialization on main()



double g_vxi;
double g_vyi;
double g_vzi;

double g_cx;
double g_cy;
double g_cz;

void readParameters(){


    // Map variables
    


    g_c = 1500; // sound velocity in water

    g_dt = 5; // time step for propagation model

    g_z_offset = 1;

    g_vxi = 0;
    g_vyi = 1;
    g_vzi = 0;

    g_cx = 1000;
    g_cy = 1000;
    g_cz = 1000;


    XMLNode xMainNode=XMLNode::openFileHelper("parameters.xml","Parameters");

    // this prints "<Condor>":
    //XMLNode xNode=xMainNode.getChildNode("maneuver");
    //printf("Application Name is: '%s'\n", xNode.getChildNode("Application").getAttribute("name"));
    
    

    const char * mane = xMainNode.getAttribute("maneuver"); 

    g_maneuver = mane;

    g_motion = std::stoi(xMainNode.getAttribute("motion"));


    g_xgridi = std::stof(xMainNode.getAttribute("xgridi"));
    g_ygridi = std::stof(xMainNode.getAttribute("ygridi"));
    g_xgridf = g_xgridi + std::stof(xMainNode.getAttribute("xgridsize"));
    g_ygridf = g_ygridi + std::stof(xMainNode.getAttribute("ygridsize"));

    g_xcenter = (g_xgridf - g_xgridi) / 2;
    g_ycenter = (g_ygridf - g_ygridi) / 2;


    //std::cout << "Maneuver: " << maneuver << std::endl;


    XMLNode xNode=xMainNode.getChildNode("RangeSensors");

    

    g_n_sensors_range = std::stoi(xNode.getAttribute("n"));
    
    

    // Range Measurement parameters
    g_eta_r = std::stof(xNode.getAttribute("eta")); // range multiplication factor
    g_mi_r = std::stof(xNode.getAttribute("mi")); // noise mean (bias)
    g_sigma_r = std::stof(xNode.getAttribute("sigma")); // constant measurement variance


    xNode=xMainNode.getChildNode("AngleSensors");

    g_n_sensors_angle = std::stoi(xNode.getAttribute("n"));



    // Angle Measurement parameters
    g_eta_a = std::stof(xNode.getAttribute("eta")); // range multiplication factor
    g_mi_a = std::stof(xNode.getAttribute("mi")); // noise mean (bias)
    g_sigma_a = std::stof(xNode.getAttribute("sigma")); // constant measurement variance



    xNode=xMainNode.getChildNode("TOASensors");

    g_n_sensors_toa = std::stoi(xNode.getAttribute("n"));




    // Angle Measurement parameters
    g_eta_toa = std::stof(xNode.getAttribute("eta")); // range multiplication factor
    g_mi_toa = std::stof(xNode.getAttribute("mi")); // noise mean (bias)
    g_sigma_toa = std::stof(xNode.getAttribute("sigma")); // constant measurement variance




    xNode=xMainNode.getChildNode("TDOASensors");

    g_n_sensors_tdoa = std::stoi(xNode.getAttribute("n"));




    // Angle Measurement parameters
    g_eta_tdoa = std::stof(xNode.getAttribute("eta")); // range multiplication factor
    g_mi_tdoa = std::stof(xNode.getAttribute("mi")); // noise mean (bias)
    g_sigma_tdoa = std::stof(xNode.getAttribute("sigma")); // constant measurement variance





    g_n_sensors_total = g_n_sensors_angle + g_n_sensors_range + g_n_sensors_toa + g_n_sensors_tdoa;



    xNode=xMainNode.getChildNode("Kalman");


    g_range_measurement_period = std::stoi(xNode.getChildNode("RangePeriod").getAttribute("period"));
    
    g_angle_measurement_period = std::stoi(xNode.getChildNode("AnglePeriod").getAttribute("period"));

    g_toa_measurement_period = std::stoi(xNode.getChildNode("TOAPeriod").getAttribute("period"));

    g_tdoa_measurement_period = std::stoi(xNode.getChildNode("TDOAPeriod").getAttribute("period"));



    g_R = Eigen::MatrixXd::Zero(3,3);

    
    
    //xNode = xNode.getChildNode("R");

    // R MATRIX

    int n = xNode.getChildNode("R").nChildNode("data");

    char *ptr;
    for (int i = 0; i < n; i++){

        g_R(i,i) = std::strtod(xNode.getChildNode("R").getChildNode("data",i).getAttribute("value"),&ptr);

    }

    //std::cout << R << std::endl;

    // Q_RANGE MATRIX


    int size = std::stoi(xNode.getChildNode("QRange").getAttribute("size")); // constant measurement variance



    g_Q_range = Eigen::MatrixXd::Zero(size,size);



    n = xNode.getChildNode("QRange").nChildNode("data");



    for (int i = 0; i < n; i++){

            g_Q_range(i,i) = std::strtod(xNode.getChildNode("QRange").getChildNode("data",i).getAttribute("value"),&ptr);

        }

    //std::cout << Q_range << std::endl;

    // Q_ANGLE MATRIX

    size = std::stoi(xNode.getChildNode("QAngle").getAttribute("size")); // constant measurement variance


    g_Q_angle = Eigen::MatrixXd::Zero(size,size);



    n = xNode.getChildNode("QAngle").nChildNode("data");



    for (int i = 0; i < n; i++){

            g_Q_angle(i,i) = std::strtod(xNode.getChildNode("QAngle").getChildNode("data",i).getAttribute("value"),&ptr);

        }



    
    // Q_TOA MATRIX


    size = std::stoi(xNode.getChildNode("QTOA").getAttribute("size")); // constant measurement variance



    g_Q_toa = Eigen::MatrixXd::Zero(size,size);



    n = xNode.getChildNode("QTOA").nChildNode("data");



    for (int i = 0; i < n; i++){

            g_Q_toa(i,i) = std::strtod(xNode.getChildNode("QTOA").getChildNode("data",i).getAttribute("value"),&ptr);

        }


    // Q_TDOA MATRIX


    size = std::stoi(xNode.getChildNode("QTDOA").getAttribute("size")); // constant measurement variance



    g_Q_tdoa = Eigen::MatrixXd::Zero(size,size);



    n = xNode.getChildNode("QTDOA").nChildNode("data");



    for (int i = 0; i < n; i++){

            g_Q_tdoa(i,i) = std::strtod(xNode.getChildNode("QTDOA").getChildNode("data",i).getAttribute("value"),&ptr);

        }


    //std::cout << Q_angle << std::endl;


    xNode=xMainNode.getChildNode("GA");


    g_pop_size = std::stoi(xNode.getAttribute("popSize")); // number of individuals in the population
    g_ga_iter = std::stoi(xNode.getAttribute("GAiter")); // number of iterations for the GA
    g_local_search_iter = std::stoi(xNode.getAttribute("LocalSearchiter")); // number of iterations for the GA

    g_local_search_version = std::stoi(xNode.getAttribute("LocalSearchVersion")); // number of iterations for the GA

    g_alpha_p = g_pop_size * std::stof(xNode.getAttribute("alphaP"));    // numer of constestants for tournament
    g_alpha_m = g_pop_size * std::stof(xNode.getAttribute("alphaM"));    // numer of individuals for mutation


    g_criteria = (xNode.getAttribute("criteria"));

    g_weigths_flag = std::stoi(xNode.getAttribute("weights")); // number of individuals in the population

}






