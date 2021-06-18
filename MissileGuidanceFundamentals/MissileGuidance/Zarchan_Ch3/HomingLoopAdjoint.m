simTime = 10;
impulse = 1;        % I.C = 1 for integrator == Impulsive input
N_prime = 4;        % Effective Navigation Ratio
T = 1;              % Time constant
Vc = 4000;          % Closing Velocity
G = @(x) x*32.2;    % Quick Lambda to convert acceleration to G's [english units]
nt = G(3);          % Target Acceleration
Vm = 3000;          % Missile Velocity
HE = deg2rad(-20);  % Initial Heading Error
h = 0.0001;         % step size
buffer = 0.000001;  % prevent divide by zero error

open("/Users/dariusmensah/Desktop/CPP_Files/MissileGuidance/Zarchan_Ch3/homingLoopSim.slx");

AdjointSim = sim("/Users/dariusmensah/Desktop/CPP_Files/MissileGuidance/Zarchan_Ch3/homingLoopSim.slx");

time = AdjointSim.tout;

MHE = AdjointSim.HeadingErrorMiss.signals.values;

MNT = AdjointSim.TargetManueverMiss.signals.values;

figure(1)
plot(time, MHE)
grid on
xlabel("Flight Time [s]")
ylabel("Miss [ft]")
title("Miss Distance due to a " + rad2deg(HE) + "° Heading Error")

figure(2)
plot(time, MNT)
grid on
xlabel("Flight Time [s]")
ylabel("Miss [ft]")
title("Miss Distance due to a " + nt/32.2 + "G Target Manuever")