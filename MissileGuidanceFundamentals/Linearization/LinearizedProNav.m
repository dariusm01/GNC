%% Similar Model to ProportionalNavigation but simplified equations
simTime = 10;
h = 0.0002;

% Closing Velocity (ft/s)
Vc = 4000;

% Target Acceleration in G's
nt = 0;

% Heading Error (rad)
HE = -20;
HE = deg2rad(HE);

% Final time
Tf = simTime;

% Navigation Ratio
N_prime = 4;

% Missile Velocity (ft/s)
Vm = 3000;

% Initial Conditions
y = 0;
yDot = -Vm*HE;

%% Running Sim
linearizedModel = sim("LinearMissileModel.slx");

%% Gathering Results
relativePos = linearizedModel.RelativeSeparation.signals.values;

accelCmmd = linearizedModel.LinearAccelG.signals.values;

time = linearizedModel.tout;

fprintf("\nThe Miss Distance is approximately %g ft\n", abs(relativePos(end)))

%% Plotting
figure(1)
plot(time,accelCmmd)
grid on
xlabel("Time (sec)")
ylabel("Missile Acceleration (G)")
title("Acceleration from Linearized Model")

% % use when nt is not 0
if nt ~= 0
    axis([0 Tf 0 max(accelCmmd)+1])
end 
