#include<iostream>
#include<cmath>
#include<vector>
#include<fstream>


template <class generic>
generic Miss_nt(generic &tf, generic &T, int &&N){

    // Miss distance due to target manuever

    generic A = pow(tf,2) * exp(-tf/T);

    if (N ==3){
        return 0.5 * A;

    }else if (N ==4){

        return A * (0.5 - (tf/(6*T)));

    }else if (N ==5){

        return A * (0.5 - (tf/(3*T)) + ((pow(tf,2))/(24*pow(T,2))));

    }else{
        return 0;
    }
}


template <class generic>
generic Miss_HE(generic &tf, generic &T, int &&N){

    // Miss distance due to heading error

    generic B = tf * exp(-tf/T);

    if (N ==3){
        return B * (1 - (tf/(2*T)));;

    }else if (N ==4){

        return B * (1 - (tf/T) + (pow(tf,2))/(6*pow(T,2)));

    }else if (N ==5){

        return B * (1 - ((3*tf)/(2*T)) + ((pow(tf,2))/(2*pow(T,2))) + ((pow(tf,3))/(24*pow(T,3))));

    }else{
        return 0;
    }
}

int main(){

    // Quick Lambda functions
    auto deg2rad = [](float &&deg){
        return deg*(M_PI/180); 
        };

    auto Gs = [](float &&accel){
        return accel*32.2;
    };

    // Creating the file
    std::ofstream out_file;
    std::string fileName;
    fileName = "AdjointClosedFormSol.txt";
    out_file.open(fileName);

    float simTime{10.0};
    float h{0.0002};
    float t{0.0};
    float tau{1};
    float buffer{0.00001}; // help prevent divide by zero error

    float HE = deg2rad(-20); // Heading Error
    float nt = Gs(3.0); //  target acceleration (in G's)
    float Vm{3000}; // Missile Velocity

    // Empty vectors
    std::vector <float> time;
    std::vector <float> MNT_3;
    std::vector <float> MNT_4;
    std::vector <float> MNT_5;
    std::vector <float> MHE_3;
    std::vector <float> MHE_4;
    std::vector <float> MHE_5;
    

    float HE_3,HE_4,HE_5,TM_3,TM_4,TM_5;

    t += buffer;

    while(t<= simTime){

        TM_3 = Miss_nt(t, tau, 3) * nt;
        TM_4 = Miss_nt(t, tau, 4) * nt;
        TM_5 = Miss_nt(t, tau, 5) * nt;

        HE_3 = Miss_HE(t, tau, 3) * (-Vm*HE);
        HE_4 = Miss_HE(t, tau, 4) * (-Vm*HE);
        HE_5 = Miss_HE(t, tau, 5) * (-Vm*HE);


        // Storing Values
        time.push_back(t);

        MNT_3.push_back(TM_3);
        MNT_4.push_back(TM_4);
        MNT_5.push_back(TM_5);

        MHE_3.push_back(HE_3);
        MHE_4.push_back(HE_4);
        MHE_5.push_back(HE_5);

        t += h;
    }

    out_file<<"Time"<<"\t"<<"MissNT3"<<"\t"<<"MissNT4"<<"\t"<<"MissNT5"<<"\t"<<"MissVM3"<<"\t"<<"MissVM4"<<"\t"<<"MissVM5"<<std::endl;


    for(int i = 0; i!=time.size(); ++i){

        out_file<<time.at(i)<<",\t"<<MNT_3.at(i)<<",\t"<<MNT_4.at(i)<<",\t"<<MNT_5.at(i)<<",\t"<<MHE_3.at(i)<<",\t"<<MHE_4.at(i)<<
        ",\t"<<MHE_5.at(i)<<std::endl;
    }

    out_file.close();


    return 0;
}