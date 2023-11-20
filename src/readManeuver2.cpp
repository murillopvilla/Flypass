
#include "../include/readManeuver.h"

// ---------- Read text file ----------

std::vector<Target> readManeuverFile2(std::string filename)
{

    std::vector<Target> tg;

    //std::vector<point> target;
    //const std::vector<point> target_const;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> phi;

    //std::vector<std::string> temp;
    double c;

    double n;

    int current_line = 0;
    std::string line;

    std::fstream file;


    file.open(filename, std::ios::in); // read
    if (file.is_open())
    {
        while (!file.eof()){
            current_line++;
            std::getline(file, line); // read the next line

            if (current_line == 1)
            {
                std::istringstream stream(line);
                while (!stream.eof()) {
                    stream >>n;
                    //x.push_back(c);               
                }          
            }


            if (current_line == 2)
            {
                std::istringstream stream(line);
                while (!stream.eof()) {
                    stream >>c;
                    x.push_back(c);               
                }          
            }
           else if (current_line == 3)
            {
                std::istringstream stream(line);
                while (!stream.eof()) {
                    stream >>c;
                    y.push_back(c);                 
                }           
            }    
            else if (current_line == 4)
            {
                std::istringstream stream(line);
                while (!stream.eof()) {
                    stream >>c;
                    phi.push_back(c);                 
                }           
            }         
       }       
    }

    file.close();

    tg.reserve(n);


    for (int i = 1; i <= n; i++){   
        Target t;
        t.tags.reserve(4);

        for (int k = 4*i-4; k < 4*i; k++){

            point p(x[k], y[k]);

            t.tags.push_back(p);

        }

        t.phi = atan2((t.tags[0].y - t.tags[1].y), (t.tags[0].x - t.tags[1].x));

        // Get boat center

        float h = sqrt(pow(t.tags[0].x - t.tags[1].x,2) + pow(t.tags[0].y - t.tags[1].y,2));
        float w = sqrt(pow(t.tags[0].x - t.tags[2].x,2) + pow(t.tags[0].y - t.tags[2].y,2));

        Eigen::Matrix2d Rotwb; //rotation from boat to world ref
        Rotwb << cos(t.phi), -sin(t.phi), sin(t.phi), cos(t.phi);
        
        Eigen::Vector2d s; //rotation from boat to world ref
        s << -h/2, w/2;

        Eigen::Vector2d s2 = Rotwb*s;

        t.xc = t.tags[0].x + s2[0];
        t.yc = t.tags[0].y + s2[1];


        tg.push_back(t);

    }


    // tg.tags.reserve(x.size());

    // for (int i = 0; i < x.size(); i++){        
    //     point p(x[i], y[i]);
    //     tg.tags.push_back(p);
    // }

    // //calculate phi

    // tg.phi = atan2((tg.tags[0].y - tg.tags[1].y), (tg.tags[0].x - tg.tags[1].x));

    // // Get boat center

    // float h = sqrt(pow(tg.tags[0].x - tg.tags[1].x,2) + pow(tg.tags[0].y - tg.tags[1].y,2));
    // float w = sqrt(pow(tg.tags[0].x - tg.tags[2].x,2) + pow(tg.tags[0].y - tg.tags[2].y,2));

    // Eigen::Matrix2d Rotwb; //rotation from boat to world ref
    // Rotwb << cos(tg.phi), -sin(tg.phi), sin(tg.phi), cos(tg.phi);
    
    // Eigen::Vector2d s; //rotation from boat to world ref
    // s << -h/2, w/2;

    // Eigen::Vector2d s2 = Rotwb*s;

    // tg.xc = tg.tags[0].x + s2[0];
    // tg.yc = tg.tags[0].y + s2[1];

    

    // // Create target points ant push to target vector
    // target.reserve(x.size());        
    // for (int i = 0; i < x.size(); i++){        
    //     point p(x[i], y[i]);
    //     target.push_back(p);        
    // }
    return tg;
}

