#ifndef SOLUTION_H
#define SOLUTION_H


#include <stdio.h>

#include <vector>
#include <math.h>
#include <eigen3/Eigen/Dense>
#include <algorithm>


#include "simParameters.h"



//Struct of a point

struct point
{
public:
    double x, y, z;
    point();
    point(double a, double b, double c);     
    point(double a, double b); 
};


struct velocities
{
public:
    double x, y, z;
    velocities();
    velocities(double a, double b, double c);   
};


// ---- Struct of a sensor


struct sensor {
public:
    point pos;
    int type;
    sensor();
    sensor(int a);    
};




// ----------------- EVALUATION METRICS --------------------------------

void ComputeFIMRangeSolution(const point &t, std::vector<sensor> &s, Eigen::Matrix3d &F);

void ComputeFIMRangeSensor(const point &t, sensor &s, Eigen::Matrix3d &F);

void ComputeFIMAngleSolution(const point &t, std::vector<sensor> &s, Eigen::Matrix3d &F);

void ComputeFIMAngleSensor(const point &t, sensor &s, Eigen::Matrix3d &F);



//  ----------------- struct of a Target --------------------------------
struct Target {
    std::vector<point> tags;
    float phi; 
    float xc, yc;   
};


// // ----------------- struct of a Solution --------------------------------


struct Solution
{

public:

    float rank;
    std::vector<sensor> sensors;
    

    void fitness(const std::vector<Target> &target);   
    void fitness2D(const std::vector<point> &target);   

};



 

#endif