simTime = 10;
h = 0.0002;

% Closing Velocity
Vc = 4000;

% Target Acceleration in G's
nt = 3;

% Heading Error
HE = 0;
HE = deg2rad(HE);

% Final time
Tf = simTime;

% Navigation Ratio
N_prime = 4;

% Missile Velocity
Vm = 3000;

% Initial Conditions
y = 0;
yDot = -Vm*HE;

% Running sim
HomingLoop = sim("ZeroLagGuidance.slx");

% Gathering Data
MissDistance = HomingLoop.MissDistance.signals.values;
time = HomingLoop.tout;

% Plotting
figure(1)
plot(time,MissDistance)
grid on 
ylabel("Relative Separation (ft)")
xlabel("Time (sec)")
title("Miss Distance via Linearized Homing Loop")

