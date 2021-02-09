%% Initial Satellite location (assuming perfect range measurements)
x = 0;
y = 0;
TF = 100;
h = 1/TF;

% Reciver 1
xR1 = 1000000; % ft
yR1 =  KMtoFT(20000); % 20000 km to ft

% Reciver 2
xR2 = 500000; % ft
yR2 = 500000; % ft

% Velocity (x)
vR1 = -14600; %ft/s
vR2 = vR1;

function a = KMtoFT(b)
c = b*100000; % converting from km to cm
a = c/30.4800; % from cm to ft
end 

