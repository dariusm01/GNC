#include<iostream>
#include<cmath>
#include<vector>
#include<fstream>

float deg2rad(float &&angle){

    float pi = M_PI;

    float conversion = pi/180.0;

    return angle*conversion;

}

float Gs(float &&accel){

    return accel*32.2;
}

float LeadAngle(float &Vt,float &Vm, float &Beta,float &Lambda){

    float a = Vt*sin(Beta+Lambda);

    return asin(a/Vm);

}

float lambdaDot(float &rtmx, float &rtmy, float &ZEM, float &tgo){

    float rtm = sqrt(pow(rtmx,2) + pow(rtmy,2));

    return ZEM/(rtm*tgo);

}

float closingVelocity(float &rtmx, float &rtmy, float &vtmx, float &vtmy){

    float rtm = sqrt(pow(rtmx,2) + pow(rtmy,2));

    float a = rtmx*vtmx;

    float b = rtmy*vtmy;

    float c = (a+b)/rtm; 

    return -1*c;

}

float ZEM_PLOS(float &rtmx, float &rtmy, float &vtmx, float &vtmy, float &tgo){

    float rtm = sqrt(pow(rtmx,2) + pow(rtmy,2));

    float a = rtmx*vtmy;

    float b = rtmy*vtmx;

    float c = (a-b)/rtm; 

    return tgo*c;

}

float command_accel(int &N, float &ZEM, float &tgo){

    float tgo2 = tgo*tgo;

    return (N*ZEM)/(tgo2);
}

int main(){

    // Creating the file
    std::ofstream out_file;
    std::string fileName;
    fileName = "ZEM_twoDMissileSim.txt";
    out_file.open(fileName);

    float simTime{10.0};
    float h{0.0002};
    float t{0.0};
    float buffer{0.00001}; // help prevent divide by zero error

    // Change these to get different results
    float HE = deg2rad(0); // Heading Error
    float nt = Gs(3.0); //  target acceleration (in G's)
    int N_prime {4}; // Navigation Ratio

    // Initial Missile values
    float Vm{3000}; // Missile Velocity
    float Rm1{0}; // down range
    float Rm2{10000}; // cross range (or altitude depending on what you're looking at)

    // Initial Target Values
    float Vt{1000}; // Target Velocity
    float Beta{0}; // Target Angular Velocity
    float Rt1{40000}; // position (x)
    float Rt2{Rm2}; // position (y)

    // Relative Position
    float Rtm1{Rt1 - Rm1};
    float Rtm2{Rt2 - Rm2};
    float LOS_lambda = atan2(Rtm2,Rtm1);

    // Lead angle
    float L = LeadAngle(Vt,Vm,Beta,LOS_lambda);

    // Initial Kinematics 
    float Vm1_0 = Vm*cos(L+HE+LOS_lambda);
    float Vm2_0 = Vm*sin(L+HE+LOS_lambda);

    float Vt1_0 = -Vt*cos(Beta);
    float Vt2_0 = Vt*sin(Beta);

    float Vtm1 = Vt1_0 - Vm1_0;
    float Vtm2 = Vt2_0 - Vm2_0;

    float Vc = closingVelocity(Rtm1,Rtm2,Vtm1,Vtm2);

    float betaDot = nt/Vt;

    float tgo{simTime};

    float ZEM_Plos = ZEM_PLOS(Rtm1,Rtm2,Vtm1,Vtm2,tgo); // zero effort miss perpendicular to the line of sight

    float lamda_rate = lambdaDot(Rtm1,Rtm2,ZEM_Plos,tgo);

    // Empty vectors
    std::vector<float> LOS;
    std::vector<float> time;
    std::vector<float> LOS_rate;
    std::vector<float> ZEM;
    std::vector<float> Accel_cmd;
    std::vector<float> closingV;


    float Rt1_new,Rt2_new,Rm1_new,Rm2_new,Vt1_new,Vt2_new,Vm1_new,Vm2_new,Beta_new,relPos,Vt1,Vt2,Vm1,Vm2,zem_plos;

    Vt1 = Vt1_0;
    Vt2 = Vt2_0;

    Vm1 = Vm1_0;
    Vm2 = Vm2_0;

    float nc = command_accel(N_prime,ZEM_Plos,tgo);

    float am1 = -nc*sin(LOS_lambda);

    float am2 = nc*cos(LOS_lambda);


    while(t< simTime){

        // Integrating commanded acceleration
        Vm1_new = Vm1 + am1*h;

        Vm2_new = Vm2 + am2*h;

        Vm1 = Vm1_new;

        Vm2 = Vm2_new;

        // Integrating missile velocity
        Rm1_new = Rm1 + Vm1*h;

        Rm2_new = Rm2 + Vm2*h;

        Rm1 = Rm1_new;

        Rm2 = Rm2_new;

        // Integrating target velocity
        Rt1_new = Rt1 + Vt1*h;

        Rt2_new = Rt2 + Vt2*h;

        Rt1 = Rt1_new;

        Rt2 = Rt2_new;

        // updating relative position
        Rtm1 = Rt1 - Rm1;

        Rtm2 = Rt2 - Rm2;

        relPos = sqrt(pow(Rtm1,2)+pow(Rtm2,2));

        // new Line of sight
        LOS_lambda = atan2(Rtm2,Rtm1);

        // target flight path angle
        Beta_new = Beta + betaDot*h;

        Beta = Beta_new;

        // target velocity
        Vt1 = -Vt*cos(Beta);

        Vt2 = Vt*sin(Beta);

        // updating relative velocity
        Vtm1 = Vt1 - Vm1;

        Vtm2 = Vt2 - Vm2;

        // Time to go 
        tgo = (simTime - t) + buffer;

        // New ZEM 
        zem_plos = ZEM_PLOS(Rtm1,Rtm2,Vtm1,Vtm2,tgo);

        // calculating LOS rate
        lamda_rate = lambdaDot(Rtm1,Rtm2,zem_plos,tgo);

        // new closing velocity
        Vc = closingVelocity(Rtm1,Rtm2,Vtm1,Vtm2);

        // Commanded acceleration
        nc = command_accel(N_prime,zem_plos,tgo);

        am1 = -nc*sin(LOS_lambda);

        am2 = nc*cos(LOS_lambda);

        t += h;

         if (relPos < 1000){
            h = 0.00002;
        }

        if (relPos < 1){
            break;
        }

        // Storing Data
        time.push_back(t);
        LOS_rate.push_back(lamda_rate);
        ZEM.push_back(zem_plos);
        Accel_cmd.push_back(nc);
        closingV.push_back(Vc);
    }

    out_file<<"Time"<<"\t"<<"LOSdot"<<"\t"<<"ZEM"<<"\t"<<"NC"<<"\t"<<"VC"<<std::endl;

    for(int i = 0; i!=time.size(); ++i){

        out_file<<time.at(i)<<",\t"<<LOS_rate.at(i)<<",\t"<<ZEM.at(i)<<",\t"<<Accel_cmd.at(i)<<",\t"<<closingV.at(i)<<std::endl;
            
    }

    out_file.close();


    return 0;


}