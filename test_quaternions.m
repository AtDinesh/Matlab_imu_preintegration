%% 
clear all;
close all;

write_to_file_const = true;
write_to_file = false;

fe = 1000;
dt = 1/fe;
N = 1;
t = (0:1/fe:N);

alpha = 5;
beta = 2;
gamma = 5;


%ox oy oz evolution in degrees (for understanding) --> converted in rad
%with * pi/180
ox = pi*sin(alpha*t*pi/180); %express angle in rad before using sinus
oy = pi*sin(beta*t*pi/180);
oz = pi*sin(gamma*t*pi/180);

%rate of turn expressed in radians/s
wx = pi*alpha*cos(alpha*t*pi/180)*pi/180;
wy = pi*beta*cos(beta*t*pi/180)*pi/180;
wz = pi*gamma*cos(gamma*t*pi/180)*pi/180;

u = [wx; wy; wz];

%% succesive integration

qi0 = [1;0;0;0];
Ri0 = [1 0 0; 0 1 0; 0 0 1];

for i=1:N*fe+1
    qi0 = qProd(qi0, v2q(u(:,i)*dt));
    
    Ri0 = Ri0 * v2R(u(:,i)*dt);
end

qi0
R2q(Ri0)
av = q2v(qi0)