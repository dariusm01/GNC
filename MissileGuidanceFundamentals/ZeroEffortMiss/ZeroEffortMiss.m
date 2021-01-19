simTime = 10;
h = 0.0002;
Tf = simTime;

% Initial Missile values
Vm = 3000; % Missile Velocity
HE = 0; % Heading Error
HE = deg2rad(HE);
Rm1 = 0; % position (x)
Rm2 = 10000; % position (y)

% Initial Target Values
Vt = 1000; % Target Velocity
Beta = 0; % Target Angular Velocity
Rt1 = 40000; % position (x)
Rt2 = Rm2; % position (y)
nt = 3; % acceleration (in G's)

% Relative Position
Rtm1 = Rt1 - Rm1;
Rtm2 = Rt2 - Rm2;
Lambda = atan2(Rtm2,Rtm1);

% Lead angle
L = LeadAngle(Vt,Vm,Beta,Lambda);

Vm1_0 = Vm*cos(L+HE+Lambda);
Vm2_0 = Vm*sin(L+HE+Lambda);

Vt1_0 = -Vt*cos(Beta);
Vt2_0 = Vt*sin(Beta);

% Navigation Ratio
N_prime = 4;

% Closing Velocity 
Vc = 4000;

% Running Sim
ZEM_sim = sim("ZEM_Example.slx");

% Gathering Results
time = ZEM_sim.tout;
LambdaDot = ZEM_sim.LOSRate.signals.values;
accelCmmd = ZEM_sim.AccelG.signals.values;
ZEM_PLOS = ZEM_sim.ZEM_PLOS.signals.values;

% Plotting
figure(1)
plot(time,LambdaDot)
grid on
xlabel("Time (s)")
ylabel("LOS Rate (rad/s)")

figure(2)
plot(time,ZEM_PLOS)
grid on
xlabel("Time (s)")
ylabel("ZEM Perp to LOS (ft)")

figure(3)
plot(time(1:end-1)/Tf,accelCmmd(1:end-1))
grid on
xlabel("t/tf")
ylabel("Missile Acceleration (G)")

function L = LeadAngle(Vt,Vm,Beta,Lambda)
a = Vt*sin(Beta+Lambda);
L = asin(a/Vm);
end 