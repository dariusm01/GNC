#include<iostream>
#include<fstream>
#include<cmath>
#include<vector>

float deg2rad(float &&angle){

    float pi = M_PI;

    float conversion = pi/180.0;

    return angle*conversion;

}

float Gs(float &&accel){

    return accel*32.2;
}

float lambdaDot(float &y, float &yd, float &vc, float &tgo){

    float num = y + (yd*tgo);

    float den = vc*pow(tgo,2);

    return num/den;

}

int main(){

    // Creating the file
    std::ofstream out_file;
    std::string fileName;
    fileName = "LinearMissileSim.txt";
    out_file.open(fileName);

    float simTime{9.0};
    float h{0.0002};
    float t{0.0};
    float buffer{0.00001}; // help prevent divide by zero error

    // Change these to get different results
    float HE = deg2rad(-20); // Heading Error
    float nt = Gs(0.0); //  target acceleration (in G's)
    int N_prime {4}; // Navigation Ratio

    // Initial Missile values
    float Vm{3000}; // Missile Velocity
    float Vc{4000}; // Closing Velocity
    float Vt{Vc-Vm}; // Target velocity for a head-on case

    // Initial Kinematics (if Heading Error)
    float y{0.0};
    float ydot{-Vm*HE};
    
    //// Initial Kinematics (if Target Maneuver)
    // float y{0.0};
    // float ydot{0.0};

    float ydot_new{0.0};
    float y_new{0.0};

    // Linearized Equations
    float tgo{simTime};

    float Beta{0}; // Target Angular Velocity

    float lamda_rate = lambdaDot(y,ydot,Vc,tgo);

    float nc = N_prime*Vc*lamda_rate;

    float Rtm{Vc*(simTime-t)};

    float LOS_lambda{y/Rtm};

    float accelMissile{nc*cos(LOS_lambda)};

    float accelTarget{nt*cos(Beta)};

    float ydd{accelTarget-accelMissile};

    // Empty vectors
    std::vector<float> Y;
    std::vector<float> YDOT;
    std::vector<float> Accel_cmd;
    std::vector<float> YDOUBLEDOT;
    std::vector<float> time;
    std::vector<float> LOS;
    std::vector<float> LOS_rate;


    while(t < simTime){

        // Integrating ydd
        ydot_new = ydot + ydd*h;

        ydot = ydot_new;

        // Integrating ydot
        y_new = y + ydot*h;

        y = y_new;

        // Time to go 
        tgo = (simTime - t) + buffer;

        // Relative position
        Rtm = Vc*tgo;

        // Line of sight
        LOS_lambda = y/Rtm;

        // LOS Rate
        lamda_rate = lambdaDot(y,ydot,Vc,tgo);

        // Commanded acceleration
        nc = N_prime*Vc*lamda_rate;

        ydd = nt - nc;

        t +=h;
        
        // Storing Data
        time.push_back(t);
        Y.push_back(y);
        YDOT.push_back(ydot);
        YDOUBLEDOT.push_back(ydd);
        Accel_cmd.push_back(nc);
        LOS.push_back(LOS_lambda);
        LOS_rate.push_back(lamda_rate);

    }

    out_file<<"Time"<<"\t"<<"Y"<<"\t"<<"YDOT"<<"\t"<<"YDOUBLEDOT"<<"\t"<<"NC"<<"\t"<<"LOS"<<"\t"<<"LOSdot"<<std::endl;

    for(int i = 0; i!=time.size(); ++i){

        out_file<<time.at(i)<<",\t"<<Y.at(i)<<",\t"<<YDOT.at(i)<<",\t"<<YDOUBLEDOT.at(i)<<",\t"<<Accel_cmd.at(i)<<",\t"<<LOS.at(i)<<
        ",\t"<<LOS_rate.at(i)<<std::endl;
            
    }

    out_file.close();


    return 0;


    
}