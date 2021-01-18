simTime = 15;
h = 0.0002;

%% Initial Missile values
Vm = 3000; % Missile Velocity
HE = 0; % Heading Error
HE = deg2rad(HE);
Rm1 = 0; % position (x)
Rm2 = 10000; % position (y)

%% Initial Target Values
Vt = 1000; % Target Velocity
Beta = 0; % Target Angular Velocity
Rt1 = 40000; % position (x)
Rt2 = Rm2; % position (y)
nt = 3; 

%% Relative Position
Rtm1 = Rt1 - Rm1;
Rtm2 = Rt2 - Rm2;
Lambda = atan2(Rtm2,Rtm1);

%% Lead angle
L = LeadAngle(Vt,Vm,Beta,Lambda);

Vm1_0 = Vm*cos(L+HE+Lambda);
Vm2_0 = Vm*sin(L+HE+Lambda);

Vt1_0 = -Vt*cos(Beta);
Vt2_0 = Vt*sin(Beta);

%% Navigation Ratio
N_prime = 4;

%% Running Sim
MissileSimulation = sim("TwoD_Missile_Target.slx");

%% For Plotting
time = MissileSimulation.tout;
MissileDistance = MissileSimulation.M1_final.signals.values/1000;
MissileAltitude = MissileSimulation.M2_final.signals.values/1000;

TargetDistance = MissileSimulation.T1_final.signals.values/1000;
TargetAltitude = MissileSimulation.T2_final.signals.values/1000;

MissileAcceleration = MissileSimulation.NC_g.signals.values;

RelativeSeparation = MissileSimulation.MissDistance.signals.values;

fprintf("\nThe Miss Distance is approximately %g ft\n", RelativeSeparation(end))

figure(1)
plot(TargetDistance,TargetAltitude)
grid on
hold on
plot(MissileDistance,MissileAltitude)
title('Two-dimensional tactical missile-target engagement simulation')
xlabel('Downrange (Kft)')
ylabel('Altitude (KFt)')
plot(MissileDistance(end),MissileAltitude(end),'k.','LineWidth',3)
lgd = legend("Target","Interceptor","Collison Point",'Location','southeast');
hold off

figure(2)
plot(time,MissileAcceleration)
grid on
title('Two-dimensional tactical missile-target engagement simulation')
xlabel('Time (sec)')
ylabel('Acceleration of missile (G)')


function L = LeadAngle(Vt,Vm,Beta,Lambda)
a = Vt*sin(Beta+Lambda);
L = asin(a/Vm);
end 