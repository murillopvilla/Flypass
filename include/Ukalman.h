#ifndef UKALMAN_H
#define UKALMAN_H




#include <vector>
#include <math.h>
#include <random>

#include "solution.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>


#include "simParameters.h"



#include "solution.h" // include just because of sensor struct. Find a better way to do this

struct TARGET_ESTIMATION {

    public:
        point pos;
        velocities vel;

        Eigen::MatrixXd Cov;


        TARGET_ESTIMATION(){};

        TARGET_ESTIMATION(double a, double b, double c, double d, double e, double f, double cx, double cy, double cz){
            this->pos.x = a;
            this->pos.y = b;
            this->pos.z = c;
            this->vel.x = d;
            this->vel.y = e;
            this->vel.z = f;           

            this -> Cov.resize(3,3);

            this -> Cov << cx,0,0,
                     0,cy,0,
                     0,0,cz;



        };

};


// -----------Simulate measurements ------------ //
void simRangeMeasurements(const point &target, std::vector<sensor> &sensors, Eigen::VectorXd &measurements) {

    std::random_device device;

    std::uniform_real_distribution<double> var(0, 1);


     int i = 0;
     for (auto &sensor : sensors) {

        if (sensor.type == RANGE_SENSOR){

            double d = sqrt(pow((sensor.pos.x - target.x),2) + pow((sensor.pos.y - target.y),2) + pow((sensor.pos.z - target.z),2));


            d = d + mi_r*(1+eta_r*d); // include bias term of the measurement noise
            
            d = d + sqrt(sigma_r)*(1+eta_r*d)*var(device); // add range measurement variance

            //double ts = d/c + sqrt(sigma_toa*var(device)); // ToA measurement
    
            measurements(i) = d;

            i++;

        }
    
    }


}



// -----------Simulate measurements ------------ //
void simAngleMeasurements(const point &target, std::vector<sensor> &sensors, Eigen::VectorXd &measurements) {

    std::random_device device;

    std::uniform_real_distribution<double> var(0, 1);


     int i = 0;
     for (auto &sensor : sensors) {

        if (sensor.type == ANGLE_SENSOR){


            double r = sqrt(pow((sensor.pos.x - target.x),2) + pow((sensor.pos.y - target.y),2) + + pow((sensor.pos.z - target.z),2));


            double r2D = sqrt(pow((sensor.pos.x - target.x),2) + pow((sensor.pos.y - target.y),2));

            // Azimuth
            //double phi = atan2((target.x - sensor.pos.x),(target.y - sensor.pos.y));

            double phi = atan2((target.y - sensor.pos.y),(target.x - sensor.pos.x));



            // Elevation

            //double theta = atan2((target.z - sensor.pos.z),r2D);

            double theta = atan((target.z - sensor.pos.z)/r2D);



            phi = phi + mi_a*(1+eta_a*r); // include bias term of the measurement noise
            
            phi = phi + sqrt(sigma_a)*(1+eta_a*r)*var(device); // add range measurement variance


            theta = theta + mi_a*(1+eta_a*r); // include bias term of the measurement noise
            
            theta = theta + sqrt(sigma_a)*(1+eta_a*r)*var(device); // add range measurement variance



            //double ts = d/c + sqrt(sigma_toa*var(device)); // ToA measurement
    
            measurements(i) = phi;

            measurements(i+1) = theta;


            i=i+2;

        }
    
    }


}


void simToAMeasurements(const point &target, std::vector<sensor> &sensors, Eigen::VectorXd &measurements) {

    std::random_device device;

    std::uniform_real_distribution<double> var(0, 1);


     int i = 0;
     for (auto &sensor : sensors) {

        if (sensor.type == TOA_SENSOR) {

            double d = sqrt(pow((sensor.pos.x - target.x),2) + pow((sensor.pos.y - target.y),2) + pow((sensor.pos.z - target.z),2));


            double t = d/c + mi_toa*(1+eta_toa*d); // include bias term of the measurement noise
            
            t = t + sqrt(sigma_toa)*(1+eta_toa*d)*var(device); // add range measurement variance

            //double ts = d/c + sqrt(sigma_toa*var(device)); // ToA measurement
    
            measurements(i) = t;

            i++;

        }
    
    }


}



// -----------Simulate measurements ------------ //
void simTDoAMeasurements(const point &target, std::vector<sensor> &sensors, Eigen::VectorXd &measurements) {

    std::random_device device;

    std::uniform_real_distribution<double> var(0, 1);


    Eigen::VectorXd Toa(nSensorsTDOA);


    int i = 0;
     for (auto &sensor : sensors) {

        if (sensor.type == TDOA_SENSOR) {

            double d = sqrt(pow((sensor.pos.x - target.x),2) + pow((sensor.pos.y - target.y),2) + pow((sensor.pos.z - target.z),2));


            double t = d/c + mi_toa*(1+eta_toa*d); // include bias term of the measurement noise
            
            t = t + sqrt(sigma_toa)*(1+eta_toa*d)*var(device); // add range measurement variance

            //double ts = d/c + sqrt(sigma_toa*var(device)); // ToA measurement
    
            Toa(i) = t;

            i++;

        }
    
    }






    Eigen::VectorXd TDoA(nSensorsTDOA-1);

     for (int i = 1; i < nSensorsTDOA; i++) {
        
    
            measurements(i-1) = Toa(0) - Toa(i);        
    
    }


}






// -----------Prediction ------------ //

void prediction(TARGET_ESTIMATION &target_est, double dt){

    //TARGET_ESTIMATION target_pred;

    target_est.pos.x = target_est.pos.x + target_est.vel.x*dt;

    target_est.pos.y = target_est.pos.y + target_est.vel.y*dt;


    target_est.pos.z = target_est.pos.z + target_est.vel.z*dt;


    //Eigen::MatrixXd G;

    //target_prev.Cov = G*target_prev.Cov*G.inverse() + R;

    target_est.Cov = target_est.Cov + R;

}

void generateSigmaPoints(TARGET_ESTIMATION &target_pred){

    int n = 3; //number of states

    int nSigma = 2*n + 1; //(2*3 + 1)

    int k = 0;
    double alpha = 1e-6;
    double lambda = pow(alpha,2)*(n+k) - n;

    int beta = 2;

    Eigen::VectorXd wm(nSigma); // weights for mean
    Eigen::VectorXd wc(nSigma); // weights for covariance


    wm(0) = lambda/(n+lambda); 

    wc(0) = wm(0) + (1-pow(alpha,2) + beta); 


    for(int i=1; i<nSigma; i++){

        wm(i) = 1/(2*(n+lambda));

        wc(i) = wm(i);

    }


    Eigen::MatrixXd L(target_pred.Cov.llt().matrixL());


}



// -----------Correction ------------ //

void updateRange(TARGET_ESTIMATION &target_pred, std::vector<sensor> &sensors, Eigen::VectorXd &measurement){


    Eigen::Vector3d state_pred {target_pred.pos.x, target_pred.pos.y, target_pred.pos.z};

    // Expected measurement (measurements are time of arrival)


    std::vector<double> d(nSensorsRange); // each measurement is stored in a vector to be reused after in the jacobian

    Eigen::VectorXd expected_measurement(nSensorsRange);


   // Eigen::Vector4d expected_measurement;



    // *** Define Measurement Jacobian *** sensors.size() x 3
    Eigen::MatrixXd H (nSensorsRange,3);


    int i = 0;
    for (auto &sensor : sensors) {

        if (sensor.type == RANGE_SENSOR){

            d[i] = sqrt(pow((sensor.pos.x - target_pred.pos.x),2) + pow((sensor.pos.y - target_pred.pos.y),2) + pow((sensor.pos.z - target_pred.pos.z),2));

            //double ts = d/c;

        
            expected_measurement(i) = d[i];




            // H(i,0) =  (target_pred.pos.x - sensors[i].pos.x)/ d[i];
            // H(i,1) =  (target_pred.pos.y - sensors[i].pos.y)/ d[i];
            // H(i,2) =  (target_pred.pos.z - sensors[i].pos.z)/ d[i];


            H(i,0) =  (target_pred.pos.x - sensor.pos.x)/ d[i];
            H(i,1) =  (target_pred.pos.y - sensor.pos.y)/ d[i];
            H(i,2) =  (target_pred.pos.z - sensor.pos.z)/ d[i];

            i++;

        }

            
    }


   

    // Inovation Covariance Matrix
    Eigen::MatrixXd S = H * target_pred.Cov * H.transpose() + Q_range;

    // Inovation
    Eigen::VectorXd inov = measurement - expected_measurement;

    // Kalman gain
    Eigen::MatrixXd K = target_pred.Cov * H.transpose() * S.inverse();

    // State correction
    Eigen::Vector3d state = state_pred + K*inov;


    target_pred.pos.x = state(0);
    target_pred.pos.y = state(1);
    target_pred.pos.z = state(2);



    // Covariance correction
    target_pred.Cov = target_pred.Cov - K*H*target_pred.Cov;


}







void updateAngle(TARGET_ESTIMATION &target_pred, std::vector<sensor> &sensors, Eigen::VectorXd &measurement){


    Eigen::Vector3d state_pred {target_pred.pos.x, target_pred.pos.y, target_pred.pos.z};

    // Expected measurement (measurements are time of arrival)


   // std::vector<double> d(nSensorsAngle); // each measurement is stored in a vector to be reused after in the jacobian



    std::vector<double> r(nSensorsAngle); // each measurement is stored in a vector to be reused after in the jacobian

    std::vector<double> r2D(nSensorsAngle); // each measurement is stored in a vector to be reused after in the jacobian

    std::vector<double> phi(nSensorsAngle); // each measurement is stored in a vector to be reused after in the jacobian

    std::vector<double> theta(nSensorsAngle); // each measurement is stored in a vector to be reused after in the jacobian


    Eigen::VectorXd expected_measurement(2*nSensorsAngle);


   // Eigen::Vector4d expected_measurement;

    // *** Define Measurement Jacobian *** 

    Eigen::MatrixXd H (nSensorsAngle*2,3);


    int i = 0;
    int j = 0;
    for (auto &sensor : sensors) {

        if (sensor.type == ANGLE_SENSOR){

          //  d[i] = sqrt(pow((sensor.pos.x - target_pred.pos.x),2) + pow((sensor.pos.y - target_pred.pos.y),2) + pow((sensor.pos.z - target_pred.pos.z),2));

            //double ts = d/c;



            r[j] = sqrt(pow((sensor.pos.x - target_pred.pos.x),2) + pow((sensor.pos.y - target_pred.pos.y),2) + pow((sensor.pos.z - target_pred.pos.z),2));


            r2D[j] = sqrt(pow((sensor.pos.x - target_pred.pos.x),2) + pow((sensor.pos.y - target_pred.pos.y),2));

            // Azimuth
            //phi[j] = atan2((target_pred.pos.x - sensor.pos.x),(target_pred.pos.y - sensor.pos.y ));

            phi[j] = atan2((target_pred.pos.y - sensor.pos.y ),(target_pred.pos.x - sensor.pos.x));


            // Elevation

            //theta[j] = atan2((target_pred.pos.z - sensor.pos.z),r2D[j]);

            theta[j] = atan((target_pred.pos.z - sensor.pos.z)/r2D[j]);

        
            expected_measurement(i) = phi[j];

            expected_measurement(i+1) = theta[j];


            H(i,0) =  (-sin(phi[j]))/ r2D[j];
            H(i,1) =  (cos(phi[j]))/ r2D[j];
            H(i,2) =  0;



            H(i+1,0) =  (-sin(theta[j])*cos(phi[j]))/ r[j];
            H(i+1,1) =  (-sin(theta[j])*sin(phi[j]))/ r[j];
            H(i+1,2) =  (cos(theta[j]))/ r[j];
            

            j ++;
            i=i+2;

        }

            
    }

    // // *** Define Measurement Jacobian *** 
    // Eigen::MatrixXd H (nSensorsAngle*2,3);

    // for (int i=0; i<nSensorsAngle; i=i+2) {

    //     H(i,0) =  (sin(phi[i]))/ r2D[i];
    //     H(i,1) =  (cos(phi[i]))/ r2D[i];
    //     H(i,2) =  0;



    //     H(i+1,0) =  (sin(theta[i])*cos(phi[i]))/ r[i];
    //     H(i+1,1) =  (sin(theta[i])*sin(phi[i]))/ r[i];
    //     H(i+1,2) =  (cos(theta[i]))/ r[i];



    // }


    // Inovation Covariance Matrix
    Eigen::MatrixXd S = H * target_pred.Cov * H.transpose() + Q_angle;

    // Inovation
    Eigen::VectorXd inov = measurement - expected_measurement;

    // Kalman gain
    Eigen::MatrixXd K = target_pred.Cov * H.transpose() * S.inverse();

    // State correction
    Eigen::Vector3d state = state_pred + K*inov;


    target_pred.pos.x = state(0);
    target_pred.pos.y = state(1);
    target_pred.pos.z = state(2);



    // Covariance correction
    target_pred.Cov = target_pred.Cov - K*H*target_pred.Cov;


}




void updateToA(TARGET_ESTIMATION &target_pred, std::vector<sensor> &sensors, Eigen::VectorXd &measurement){


    Eigen::Vector3d state_pred {target_pred.pos.x, target_pred.pos.y, target_pred.pos.z};

    // Expected measurement (measurements are time of arrival)


    std::vector<double> d(nSensorsTOA); // each measurement is stored in a vector to be reused after in the jacobian

    Eigen::VectorXd expected_measurement(nSensorsTOA);


   // Eigen::Vector4d expected_measurement;



    // *** Define Measurement Jacobian *** sensors.size() x 3
    Eigen::MatrixXd H (nSensorsTOA,3);


    int i = 0;
    for (auto &sensor : sensors) {

        if (sensor.type == TOA_SENSOR){

            d[i] = sqrt(pow((sensor.pos.x - target_pred.pos.x),2) + pow((sensor.pos.y - target_pred.pos.y),2) + pow((sensor.pos.z - target_pred.pos.z),2));

            //double ts = d/c;
        
            expected_measurement(i) = d[i]/c;


            H(i,0) =  (target_pred.pos.x - sensor.pos.x)/ (c*d[i]);
            H(i,1) =  (target_pred.pos.y - sensor.pos.y)/ (c*d[i]);
            H(i,2) =  (target_pred.pos.z - sensor.pos.z)/ (c*d[i]);



            i++;

        }

            
    }

   
    // Inovation Covariance Matrix
    Eigen::MatrixXd S = H * target_pred.Cov * H.transpose() + Q_toa;

    // Inovation
    Eigen::VectorXd inov = measurement - expected_measurement;

    // Kalman gain
    Eigen::MatrixXd K = target_pred.Cov * H.transpose() * S.inverse();

    // State correction
    Eigen::Vector3d state = state_pred + K*inov;


    target_pred.pos.x = state(0);
    target_pred.pos.y = state(1);
    target_pred.pos.z = state(2);


    // Covariance correction
    target_pred.Cov = target_pred.Cov - K*H*target_pred.Cov;


}



void updateTDoA(TARGET_ESTIMATION &target_pred, std::vector<sensor> &sensors, Eigen::VectorXd &measurement){


    Eigen::Vector3d state_pred {target_pred.pos.x, target_pred.pos.y, target_pred.pos.z};

    // Expected measurement (measurements are time of arrival)


    std::vector<double> d(nSensorsTDOA); // each measurement is stored in a vector to be reused after in the jacobian



    Eigen::VectorXd expected_ToA(nSensorsTDOA);


    Eigen::VectorXd expected_measurement(nSensorsTDOA - 1);


   // Eigen::Vector4d expected_measurement;



    // *** Define Measurement Jacobian *** sensors.size() x 3
    Eigen::MatrixXd H (nSensorsTDOA - 1,3);





    std::vector<sensor> sensorsTDOA; // to store position of sensors of TDOA

    sensorsTDOA.reserve(nSensorsTDOA);



    int i = 0;
    for (auto &sensor : sensors) {

        if (sensor.type == TDOA_SENSOR){

            d[i] = sqrt(pow((sensor.pos.x - target_pred.pos.x),2) + pow((sensor.pos.y - target_pred.pos.y),2) + pow((sensor.pos.z - target_pred.pos.z),2));

            //double ts = d/c;
        
            expected_ToA(i) = d[i]/c;



            sensorsTDOA[i].pos.x = sensor.pos.x;
            sensorsTDOA[i].pos.y = sensor.pos.y;
            sensorsTDOA[i].pos.z = sensor.pos.z;



            
            i++;

        }

            
    }

    for (int i=1; i<expected_measurement.size();i++) {

        
        expected_measurement(i-1) = expected_ToA(0)-expected_ToA(i);


    
        H(i-1,0) =  ((target_pred.pos.x - sensorsTDOA[0].pos.x)/ (c*d[0])) - ((target_pred.pos.x - sensorsTDOA[i].pos.x)/ (c*d[i]));
        H(i-1,1) =  ((target_pred.pos.y - sensorsTDOA[0].pos.y)/ (c*d[0])) - ((target_pred.pos.y - sensorsTDOA[i].pos.y)/ (c*d[i]));
        H(i-1,2) =  ((target_pred.pos.z - sensorsTDOA[0].pos.z)/ (c*d[0])) - ((target_pred.pos.z - sensorsTDOA[i].pos.z)/ (c*d[i]));

    }

   
    // Inovation Covariance Matrix
    Eigen::MatrixXd S = H * target_pred.Cov * H.transpose() + Q_tdoa;

    // Inovation
    Eigen::VectorXd inov = measurement - expected_measurement;

    // Kalman gain
    Eigen::MatrixXd K = target_pred.Cov * H.transpose() * S.inverse();

    // State correction
    Eigen::Vector3d state = state_pred + K*inov;


    target_pred.pos.x = state(0);
    target_pred.pos.y = state(1);
    target_pred.pos.z = state(2);


    // Covariance correction
    target_pred.Cov = target_pred.Cov - K*H*target_pred.Cov;


}


void runKalman(const std::vector<point> &targetpos, std::vector<sensor> &sensors, std::vector<TARGET_ESTIMATION> &target_estimations, 
TARGET_ESTIMATION &target_est,  int measurement_flag){   

   
    // ---------- Estimate the target position --------

    // Prediction 
    
    prediction(target_est, dt);
    
    

    if ((measurement_flag % range_measurement_period == 0) && (nSensorsRange != 0)){

        // Simulate the measurements

        Eigen::VectorXd measurements(nSensorsRange);

        

        simRangeMeasurements(targetpos[0], sensors, measurements);

        
        // Update
        updateRange(target_est, sensors, measurements);

    }
        

    if (measurement_flag % angle_measurement_period == 0 && (nSensorsAngle != 0)){

        // Simulate the measurements

        Eigen::VectorXd measurements(2*nSensorsAngle);

        

        simAngleMeasurements(targetpos[0], sensors, measurements);

        
        // Update
        updateAngle(target_est, sensors, measurements);
        

    }



    if ((measurement_flag % toa_measurement_period == 0) && (nSensorsTOA != 0)){

        // Simulate the measurements

        Eigen::VectorXd measurements(nSensorsTOA);

        

        simToAMeasurements(targetpos[0], sensors, measurements);

        
        // Update
        updateToA(target_est, sensors, measurements);

    }




    if ((measurement_flag % tdoa_measurement_period == 0) && (nSensorsTDOA != 0)){

        // Simulate the measurements

        Eigen::VectorXd measurements(nSensorsTDOA-1);

        

        simTDoAMeasurements(targetpos[0], sensors, measurements);

        
        // Update
        updateTDoA(target_est, sensors, measurements);

    }



    // Put estimation in the estimation vector
    target_estimations.push_back(target_est);




}   



#endif