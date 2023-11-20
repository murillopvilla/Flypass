#ifndef EKALMAN_H
#define EKALMAN_H




#include <vector>
#include <math.h>
#include <random>

#include "solution.h"
#include <eigen3/Eigen/Dense>

#include "simParameters.h"



#include "solution.h" // include just because of sensor struct. Find a better way to do this

struct TargetEstimation {

    public:
        point pos;
        velocities vel;

        Eigen::MatrixXd Cov;


        TargetEstimation();

        TargetEstimation(double a, double b, double c, double d, double e, double f, double cx, double cy, double cz);

};


// -----------Simulate measurements ------------ //
void simRangeMeasurements(const point &target, std::vector<sensor> &sensors, Eigen::VectorXd &measurements);


void simAngleMeasurements(const point &target, std::vector<sensor> &sensors, Eigen::VectorXd &measurements);


void simToAMeasurements(const point &target, std::vector<sensor> &sensors, Eigen::VectorXd &measurements);


void simTDoAMeasurements(const point &target, std::vector<sensor> &sensors, Eigen::VectorXd &measurements);





// -----------Prediction ------------ //

void prediction(TargetEstimation &target_est, double dt);




// -----------Correction ------------ //

void updateRange(TargetEstimation &target_pred, std::vector<sensor> &sensors, Eigen::VectorXd &measurement);


void updateAngle(TargetEstimation &target_pred, std::vector<sensor> &sensors, Eigen::VectorXd &measurement);


void updateToA(TargetEstimation &target_pred, std::vector<sensor> &sensors, Eigen::VectorXd &measurement);

void updateTDoA(TargetEstimation &target_pred, std::vector<sensor> &sensors, Eigen::VectorXd &measurement);


// -----------main function ------------ //

void runKalman(const std::vector<point> &targetpos, std::vector<sensor> &sensors, std::vector<TargetEstimation> &target_estimations, 
TargetEstimation &target_est,  int measurement_flag);

#endif