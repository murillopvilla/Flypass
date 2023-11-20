
#include "../include/GA.h"





// Random Solution Initialization

void randomSolInit(std::vector<Solution> &solvec, int localpopsize){
    std::random_device device;
    std::uniform_real_distribution<> unifx(g_xgridi, g_xgridf);
    std::uniform_real_distribution<> unify(g_xgridf, g_ygridf);
    for (int i = 0; i < localpopsize; i++)
    { // loop over the population size
        Solution S;
        S.sensors.reserve(g_n_sensors_total);
        for (int j = 0; j < g_n_sensors_range; j++) // loop over the number of sensors of range
        {
            sensor s;
            s.type = RANGE_SENSOR;
            point p(unifx(device), unify(device), 0); // generate random sensor coordinates. Z coordinates are zero, as sensors are in the plane
            s.pos = p;
            S.sensors.push_back(s); // add sensor to the solution
        }

         for (int j = 0; j < g_n_sensors_angle; j++) // loop over the number of sensors of angle
        {
            sensor s;
            s.type = ANGLE_SENSOR;
            point p(unifx(device), unify(device), 0); // generate random sensor coordinates. Z coordinates are zero, as sensors are in the plane
            s.pos = p;
            S.sensors.push_back(s); // add sensor to the solution
        }
        solvec.push_back(S); // add solution to the solution vector
    }    
}






// ---------- GA DECLARATIONS ------------------- //


void CrossOver(std::vector<Solution> &P, std::vector<Solution> &children)
{
    std::vector<float> r;
    r.reserve(P.size()*(g_n_sensors_angle + g_n_sensors_range));
    std::vector<float> angle;
    angle.reserve(P.size()*(g_n_sensors_angle + g_n_sensors_range));

    // Iterate over parents and calculate distance and angle to the center of the plane
    for (auto &Solution : P)
    { // for each solution in the parents vector
        for (auto &sensor : Solution.sensors)
        { // for each sensor in the solution
            // Calculate distance and angle to the center of the plane
            r.push_back(std::sqrt(pow((sensor.pos.x - g_xcenter), 2) + pow((sensor.pos.y - g_ycenter), 2)));
            angle.push_back(std::atan2((sensor.pos.y - g_ycenter), (sensor.pos.x - g_xcenter)));
        }
    }

    for (size_t i = 0; i < P.size(); i += 2)
    {
        int sensor_number = 0;
        // Create children type 1 (distance of parent 1 and angle of parent 2)
        for (auto &sensor : children[i].sensors)
        {
            if(sensor_number == (g_n_sensors_angle + g_n_sensors_range)) sensor_number = 0;
            sensor.pos.x = r[i*g_n_sensors_total + sensor_number] * cos(angle[i*g_n_sensors_total + g_n_sensors_total + sensor_number]) + g_xcenter;
            sensor.pos.y = r[i*g_n_sensors_total + sensor_number] * sin(angle[i*g_n_sensors_total + g_n_sensors_total + sensor_number]) + g_ycenter;
            sensor.pos.z = 0;
            // Check boundaries
            if ((sensor).pos.x < g_xgridi)
                (sensor).pos.x = g_xgridi;
            else if ((sensor).pos.x > g_xgridf)
                (sensor).pos.x = g_xgridf;

            if ((sensor).pos.y < g_ygridi)
                (sensor).pos.y = g_ygridi;
            else if ((sensor).pos.y > g_ygridf)
                (sensor).pos.y = g_ygridf;

            sensor_number++;

        }

        // Create children type 2 (distance of parent 2 and angle of parent 1)

        for (auto &sensor : children[i + 1].sensors)
        {
            if(sensor_number == g_n_sensors_total) sensor_number = 0;
            sensor.pos.x = r[i*g_n_sensors_total + g_n_sensors_total + sensor_number] * cos(angle[i*g_n_sensors_total+ sensor_number]) + g_xcenter;
            sensor.pos.y = r[i*g_n_sensors_total + g_n_sensors_total + sensor_number] * sin(angle[i*g_n_sensors_total+ sensor_number]) + g_ycenter;
            sensor.pos.z = 0;

            // Check boundaries
            if ((sensor).pos.x < g_xgridi)
                (sensor).pos.x = g_xgridi;
            else if ((sensor).pos.x > g_xgridf)
                (sensor).pos.x = g_xgridf;

            if ((sensor).pos.y < g_ygridi)
                (sensor).pos.y = g_ygridi;
            else if ((sensor).pos.y > g_ygridf)
                (sensor).pos.y = g_ygridf;
            sensor_number++;
        }
    }
}

void Mutate(std::vector<Solution> &S, std::size_t c, const std::vector<Target> &target)
{

    std::random_device device;
    std::uniform_int_distribution<> solution(0, g_pop_size - c - 1);
    std::uniform_int_distribution<> sens(0, g_n_sensors_total - 1);
    std::uniform_real_distribution<double> ang(0, 1);
    sensor *sensor; // pointer to a random sensor of a random solution
    for (int i = 0; i < g_alpha_m; i++)
    {
        float angle = ang(device) * 2 * M_PI; // generate random direction
        float r = 0.01;
        float x = r * cos(angle);
        float y = r * sin(angle);
        int randomSolution = solution(device);
        int randomSensor = sens(device);
        sensor = &S[randomSolution].sensors[randomSensor]; // pointer to a random sensor of a random solution
        point p(x + (*sensor).pos.x, y + (*sensor).pos.y, 0); // create a new point for the random sensor
        (*sensor).pos = p;
        // Check boundaries

        if ((*sensor).pos.x < g_xgridi)
            (*sensor).pos.x = g_xgridi;
        else if ((*sensor).pos.x > g_xgridf)
            (*sensor).pos.x = g_xgridf;

        if ((*sensor).pos.y < g_ygridi)
            (*sensor).pos.y = g_ygridi;
        else if ((*sensor).pos.y > g_ygridf)
            (*sensor).pos.y = g_ygridf;
        // Calculate rank of the new solution
        S[randomSolution].fitness(target);
        
    }
}










