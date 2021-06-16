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

float lambdaDot(float &rtmx, float &rtmy, float &vtmx, float &vtmy){

    float tm2 = pow(rtmx,2) + pow(rtmy,2);

    float a = rtmx*vtmy;

    float b = rtmy*vtmx;

    return (a-b)/tm2;
}

float closingVelocity(float &rtmx, float &rtmy, float &vtmx, float &vtmy){

    float tm = sqrt(pow(rtmx,2) + pow(rtmy,2));

    float a = rtmx*vtmx;

    float b = rtmy*vtmy;

    float c = (a+b)/tm; 

    return -1*c;

}

int main(){

    // Creating the file
    std::ofstream out_file;
    std::string fileName;
    fileName = "twoDMissileSim.txt";
    out_file.open(fileName);

    int simTime{15};
    float h{0.0002};
    float t{0.0};

    // Change these to get different results
    float HE = deg2rad(-20); // Heading Error
    float nt = Gs(0.0); //  target acceleration (in G's)
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

    float lamda_rate = lambdaDot(Rtm1,Rtm2,Vtm1,Vtm2);

    float betaDot = nt/Vt;


    // Empty vectors
    std::vector<float> LOS;
    std::vector<float> Range;
    std::vector<float> time;
    std::vector<float> LOS_rate;
    std::vector<float> closingV;
    std::vector<float> Accel_cmd;
    std::vector<float> Rtx;
    std::vector<float> Rty;
    std::vector<float> Rmx;
    std::vector<float> Rmy;

    float Rt1_new,Rt2_new,Rm1_new,Rm2_new,Vt1_new,Vt2_new,Vm1_new,Vm2_new,Beta_new,relPos,Vt1,Vt2,Vm1,Vm2;

    Vt1 = Vt1_0;
    Vt2 = Vt2_0;

    Vm1 = Vm1_0;
    Vm2 = Vm2_0;

    float nc = N_prime*Vc*lamda_rate;

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

        // calculating LOS rate
        lamda_rate = lambdaDot(Rtm1,Rtm2,Vtm1,Vtm2);

        // new closing velocity
        Vc = closingVelocity(Rtm1,Rtm2,Vtm1,Vtm2);

        // Commanded acceleration
        nc = N_prime*Vc*lamda_rate;

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
        Range.push_back(relPos);
        LOS.push_back(LOS_lambda);
        time.push_back(t);
        LOS_rate.push_back(lamda_rate);
        closingV.push_back(Vc);
        Accel_cmd.push_back(nc);
        Rtx.push_back(Rt1);
        Rty.push_back(Rt2);
        Rmx.push_back(Rm1);
        Rmy.push_back(Rm2);

    }

    out_file<<"Time"<<"\t"<<"RTM"<<"\t"<<"LOS"<<"\t"<<"LOSdot"<<"\t"<<"VC"<<"\t"<<"NC"<<"\t"<<"RT1"<<"\t"<<"RT2"<<"\t"<<"RM1"<<"\t"<<"RM2"<<std::endl;

    for(int i = 0; i!=time.size(); ++i){

        out_file<<time.at(i)<<",\t"<<Range.at(i)<<",\t"<<LOS.at(i)<<",\t"<<LOS_rate.at(i)<<",\t"<<closingV.at(i)<<",\t"<<Accel_cmd.at(i)<<
        ",\t"<<Rtx.at(i)<<",\t"<<Rty.at(i)<<",\t"<<Rmx.at(i)<<",\t"<<Rmy.at(i)<<std::endl;
            
    }

    out_file.close();


    return 0;


}