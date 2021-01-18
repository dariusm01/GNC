load("Nav4.mat");
load("Nav5.mat");

%% Plotting Position of Target and Interceptor
figure(1)
plot(N4_TargetDistance,N4_TargetAltitude)
grid on
hold on
plot(N4_MissileDistance,N4_MissileAltitude)
title('Navigation Ratio = 4')
xlabel('Downrange (Kft)')
ylabel('Altitude (KFt)')
plot(N4_MissileDistance(end),N4_MissileAltitude(end),'k.','LineWidth',3)
lgd = legend("Target","Interceptor","Collison Point",'Location','southeast');
hold off

figure(2)
plot(N5_TargetDistance,N5_TargetAltitude)
grid on
hold on
plot(N5_MissileDistance,N5_MissileAltitude)
title('Navigation Ratio = 5')
xlabel('Downrange (Kft)')
ylabel('Altitude (KFt)')
plot(N5_MissileDistance(end),N5_MissileAltitude(end),'k.','LineWidth',3)
lgnd = legend("Target","Interceptor","Collison Point",'Location','southeast');
hold off

%% Plotting Acceleration in G's
figure(3)
plot(N4_time,N4_MissileAcceleration)
grid on
title('Accleration N = 4')
xlabel('Time (sec)')
ylabel('Acceleration of missile (G)')

figure(4)
plot(N5_time,N5_MissileAcceleration)
grid on
title('Accleration N = 5')
xlabel('Time (sec)')
ylabel('Acceleration of missile (G)')