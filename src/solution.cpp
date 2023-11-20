

#include "../include/solution.h"



point::point(){};


point::point(double a, double b, double c)
{
    this->x = a;
    this->y = b;
    this->z = c;
};

point::point(double a, double b)
{
    this->x = a;
    this->y = b;
    this->z = 0;
    
};

velocities::velocities(){};

velocities::velocities(double a, double b, double c)
{
    this->x = a;
    this->y = b;
    this->z = c;
};

 

sensor::sensor(int a)
    {
        this->type = a;
    };

sensor::sensor() {};




// ----------------- EVALUATION METRICS --------------------------------

void ComputeFIMRangeSensor(const point &t, sensor &s, Eigen::Matrix3d &F)
{
    //F = Eigen::Matrix3d::Zero();
    Eigen::Vector3d r;
    r << (s.pos.x - t.x), (s.pos.y - t.y), (s.pos.z - t.z);
    double std_dev = (1 + g_eta_r * r.norm());
    r = r / r.norm();     
    F = F + (1 / pow(std_dev, 2)) * (r * r.transpose());          

}

void ComputeFIMRangeSensor2D(const point &t, sensor &s, Eigen::Matrix2d &F){    //F = Eigen::Matrix3d::Zero();
    Eigen::Vector2d r;
    r << (s.pos.x - t.x), (s.pos.y - t.y);
    double std_dev = (1 + g_eta_r * r.norm());
    r = r / r.norm();     
    F = F + (1 / pow(std_dev, 2)) * (r * r.transpose());     
}

void ComputeFIM3D(const Target &target, sensor &anchor, Eigen::Matrix3d &F){    //F = Eigen::Matrix3d::Zero();
    
    for (auto tag:target.tags){
    
        Eigen::Vector3d tau;
        tau << (anchor.pos.x - tag.x), (anchor.pos.y - tag.y), 0;
        double std_dev = (1 + g_eta_r * tau.norm());
        //tau = tau / tau.norm();     

        Eigen::Vector2d naf;
        Eigen::Vector2d ntf;
        Eigen::Vector2d ntb;
        Eigen::Vector2d nbf;

        Eigen::Matrix2d Rotwb; //rotation from boat to world ref
        Eigen::Matrix2d dRotwbdphi;


        Rotwb << cos(target.phi), -sin(target.phi), sin(target.phi), cos(target.phi);

        dRotwbdphi << -sin(target.phi), -cos(target.phi), cos(target.phi), -sin(target.phi);



        naf << (anchor.pos.x - g_xgridi),(anchor.pos.y - g_ygridi); //world ref

        ntf << (tag.x - g_xgridi),(tag.y - g_ygridi); //world ref

        ntb << (tag.x - target.xc),(tag.y - target.yc); //world ref

        ntb = Rotwb.inverse()*ntb; // boat ref

        nbf << (target.xc - g_xgridi),(target.yc - g_ygridi); //world ref


        tau[2] = ((dRotwbdphi*ntb).transpose() * (nbf + Rotwb*ntb - naf)); 

        tau = tau / (ntf - naf).norm();





        
        F = F + (1 / pow(std_dev, 2)) * (tau * tau.transpose());     
    }
}

int checkSensorsAligned(std::vector<sensor> sensors){

    if((sensors[0].pos.x == sensors[1].pos.x) && (sensors[0].pos.x == sensors[2].pos.x) && (sensors[0].pos.x == sensors[3].pos.x))
        return 1;

    else if ((sensors[0].pos.y == sensors[1].pos.y) && (sensors[0].pos.y == sensors[2].pos.y) && (sensors[0].pos.y == sensors[3].pos.y))
        return 2;

    else return 0;

};



void Solution::fitness2D(const std::vector<point> &target)
    {
        rank = 0;
       // std::vector<point>::const_iterator t;
        // Iterate over all target points
       for (auto const  &t : target){
            // Compute FIM of each sensor
            Eigen::Matrix2d FIM;
            FIM = Eigen::Matrix2d::Zero();           
            for (auto &sensor : sensors){         
                if (sensor.type == RANGE_SENSOR)                        
                    ComputeFIMRangeSensor2D(t, sensor, FIM);                 
            }
            if (g_n_sensors_range != 0){
                double Theta = 2 * pow(g_eta_r, 2) + (pow(1 + g_eta_r * g_mi_r, 2)) / pow(g_sigma_r, 2); // termo só existe se existirem sensores de range
                FIM = Theta * FIM; 
            }
            // std::cout << "Here is the inverse of the FIM:\n" << FIM.inverse() << std::endl;
            //Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(FIM.inverse());

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(FIM.inverse());

            // Check if eigenvalues are computable
            if (eigensolver.info() != Eigen::Success)
                {
                   int ckal = checkSensorsAligned(sensors);
                
                    if (ckal == 1){
                        sensors[0].pos.x = sensors[0].pos.x + 0.1;  
                        fitness2D(target);
                        }               
                    else if (ckal == 2){
                        sensors[0].pos.y = sensors[0].pos.y + 0.1;  
                        fitness2D(target); 
                    }              
                    else
                    abort();
                }
                
            Eigen::Vector2d eigenvalues;
            eigenvalues = eigensolver.eigenvalues(); // Calculate eigenvalues   

            float weights[] = {0.4, 0.25, 0.2, 0.15 };

            float d = sqrt(pow((t.x-g_xgridi),2) + pow((t.y-g_ygridi),2));

            
            if(g_criteria == "E"){
                if (g_weigths_flag == 1){
                    if (d <=10)
                        rank += weights[0]*eigenvalues.maxCoeff();  // get max eigenvalue
                    else if ((d >10) && d <= 20)
                        rank += weights[1]*eigenvalues.maxCoeff();  // get max eigenvalue
                    else if ((d >20) && d <= 30)
                        rank += weights[2]*eigenvalues.maxCoeff();  // get max eigenvalue
                    else if (d > 30)
                        rank += weights[3]*eigenvalues.maxCoeff();  // get max eigenvalue
                }
                else
                rank += eigenvalues.maxCoeff();  // get max eigenvalue
            }
            else if (g_criteria == "D"){

                if (g_weigths_flag == 1){
                    if (d <=10)
                        rank += weights[0]*(eigenvalues.maxCoeff() * eigenvalues.minCoeff()); // get max eigenvalue
                    else if ((d >10) && d <= 20)
                        rank += weights[1]*(eigenvalues.maxCoeff() * eigenvalues.minCoeff()); // get max eigenvalue
                    else if ((d >20) && d <= 30)
                        rank += weights[2]*(eigenvalues.maxCoeff() * eigenvalues.minCoeff()); // get max eigenvalue
                    else if (d > 30)
                        rank += weights[3]*(eigenvalues.maxCoeff() * eigenvalues.minCoeff()); // get max eigenvalue
                }
                else
                    rank += (eigenvalues.maxCoeff() * eigenvalues.minCoeff()); // get max eigenvalue

            }

            else if (g_criteria == "A"){
                if (g_weigths_flag == 1){
                    if (d <=10)
                        rank += weights[0]*(eigenvalues.maxCoeff() + eigenvalues.minCoeff()); // get max eigenvalue
                    else if ((d >10) && d <= 20)
                        rank += weights[1]*(eigenvalues.maxCoeff() + eigenvalues.minCoeff()); // get max eigenvalue
                    else if ((d >20) && d <= 30)
                        rank += weights[2]*(eigenvalues.maxCoeff() + eigenvalues.minCoeff()); // get max eigenvalue
                    else if (d > 30)
                        rank += weights[3]*(eigenvalues.maxCoeff() + eigenvalues.minCoeff()); // get max eigenvalue
                }
                else
                    rank += (eigenvalues.maxCoeff() + eigenvalues.minCoeff()); // get max eigenvalue

            }
            // if (eigenvalues.minCoeff() < 0.001){

            //     rank = rank*2;
            // }
        };
        rank = rank / target.size();
        // std::cout << "rank: " << rank << std::endl;
    }



void Solution::fitness(const std::vector<Target> &target)
    {
        rank = 0;
       // Compute FIM of each sensor
       
       for (auto const  &t : target){  


         Eigen::Matrix3d FIM;
        FIM = Eigen::Matrix3d::Zero();                     
            for (auto &anchor : sensors) ComputeFIM3D(t, anchor, FIM);                 
            
            if (g_n_sensors_range != 0){
                double Theta = 2 * pow(g_eta_r, 2) + (pow(1 + g_eta_r * g_mi_r, 2)) / pow(g_sigma_r, 2); // termo só existe se existirem sensores de range
                FIM = Theta * FIM; 
            }
            
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(FIM.inverse());

            // Check if eigenvalues are computable
            if (eigensolver.info() != Eigen::Success)
                {
                   int ckal = checkSensorsAligned(sensors);
                
                    if (ckal == 1){
                        sensors[0].pos.x = sensors[0].pos.x + 0.1;  
                        fitness(target);
                        }               
                    else if (ckal == 2){
                        sensors[0].pos.y = sensors[0].pos.y + 0.1;  
                        fitness(target); 
                    }              
                    else
                    abort();
                }
                
            Eigen::Vector3d eigenvalues;
            eigenvalues = eigensolver.eigenvalues(); // Calculate eigenvalues   

           // float weights[] = {0.4, 0.25, 0.2, 0.15 };

           // float d = sqrt(pow((target.tags.x-g_xgridi),2) + pow((t.y-g_ygridi),2));

            
            if(g_criteria == "E"){
                // if (g_weigths_flag == 1){
                //     if (d <=10)
                //         rank += weights[0]*eigenvalues.maxCoeff();  // get max eigenvalue
                //     else if ((d >10) && d <= 20)
                //         rank += weights[1]*eigenvalues.maxCoeff();  // get max eigenvalue
                //     else if ((d >20) && d <= 30)
                //         rank += weights[2]*eigenvalues.maxCoeff();  // get max eigenvalue
                //     else if (d > 30)
                //         rank += weights[3]*eigenvalues.maxCoeff();  // get max eigenvalue
                // }
                // else
                rank += eigenvalues.maxCoeff();  // get max eigenvalue
            }
            else if (g_criteria == "D"){

                // if (g_weigths_flag == 1){
                //     if (d <=10)
                //         rank += weights[0]*(eigenvalues.maxCoeff() * eigenvalues.minCoeff()); // get max eigenvalue
                //     else if ((d >10) && d <= 20)
                //         rank += weights[1]*(eigenvalues.maxCoeff() * eigenvalues.minCoeff()); // get max eigenvalue
                //     else if ((d >20) && d <= 30)
                //         rank += weights[2]*(eigenvalues.maxCoeff() * eigenvalues.minCoeff()); // get max eigenvalue
                //     else if (d > 30)
                //         rank += weights[3]*(eigenvalues.maxCoeff() * eigenvalues.minCoeff()); // get max eigenvalue
                // }
                // else
                    rank += eigenvalues[0]*eigenvalues[1]*eigenvalues[2]; //(eigenvalues.maxCoeff() * eigenvalues.minCoeff()); // get max eigenvalue

            }

            else if (g_criteria == "A"){
                // if (g_weigths_flag == 1){
                //     if (d <=10)
                //         rank += weights[0]*(eigenvalues.maxCoeff() + eigenvalues.minCoeff()); // get max eigenvalue
                //     else if ((d >10) && d <= 20)
                //         rank += weights[1]*(eigenvalues.maxCoeff() + eigenvalues.minCoeff()); // get max eigenvalue
                //     else if ((d >20) && d <= 30)
                //         rank += weights[2]*(eigenvalues.maxCoeff() + eigenvalues.minCoeff()); // get max eigenvalue
                //     else if (d > 30)
                //         rank += weights[3]*(eigenvalues.maxCoeff() + eigenvalues.minCoeff()); // get max eigenvalue
                // }
                // else
                    rank += eigenvalues[0]+eigenvalues[1]+eigenvalues[2]; // (eigenvalues.maxCoeff() + eigenvalues.minCoeff()); // get max eigenvalue

            }
            // if (eigenvalues.minCoeff() < 0.001){

            //     rank = rank*2;
            // }
        };
        rank = rank / target.size();
        // std::cout << "rank: " << rank << std::endl;
    }
