%% 
% Testing rotations.
% Generate a rotation angle on x, y and z axes (ox, oy, oz in radians). These angles change over 
% time.
% Rotation vector (ox;oy;oz) --> Rotation Matrix R --> R*[1;0;0].

% This gives the orientation of a point that would be initially on (1;0;0);
% 
% From formulas for ox, oy and oz, get wx, wy, wz that matches the rate of turn we will give to IMU.
% Compute data using the imu_integrator with ax=ay=az=0 (pure rotation here).

%*****************************WORKING*********************

%% 
clear all;
close all;

fe = 1000;
N = 100*1;
t = (0:1/fe:N-1/fe);

r = 1;
alpha = 10;
beta = 2;
gamma = 4;

ox = alpha*t*pi/180;
oy = beta*t*pi/180;
oz = gamma*t*pi/180;

wx = alpha*pi/180;
wy = beta*pi/180;
wz = gamma*pi/180;
init_pos = [1;0;0];
pos = [1;0;0];

total_pos = [];
v = [ox(1,:); oy(1,:); oz(1,:)]; %this is the evolution of rotations

for ii= 1:size(v,2)
    Rot = v2R(v(:,ii));
    pos = Rot*init_pos;
    total_pos = [total_pos pos];
end

figure;
plot3(total_pos(1,:), total_pos(2,:), total_pos(3,:));
hold on;
plot3(0,0,0,'r*');
plot3(1,0,0,'g*');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');

%% Test

fe = 1000;
N = 100*1;
t = (0:1/fe:N-1/fe);

ax(1,1:(N*fe)) = 0;
ay(1,1:(N*fe)) = 0;
az(1,1:(N*fe)) = 0;
wx(1,1:(N*fe)) = wx; 
wy(1,1:(N*fe)) = wy;
wz(1,1:(N*fe)) = wz;
u = [ax; ay; az; wx; wy; wz];

%% needed parameters

dt = 1/fe;
di = [1; 0; 0; 1; 0; 0; 0; 0; 0; 0];
di0 = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
%u = [10; 5; 2; 110; 30; 50];

b0 = [0; 0; 0; 0; 0; 0]; %bias vector

n_ax = 0.04*randn(1,(N*fe));
n_ay = 0.04*randn(1,(N*fe));
n_az = 0.04*randn(1,(N*fe));
n_wx = 0.002*randn(1,(N*fe));
n_wy = 0.002*randn(1,(N*fe));
n_wz = 0.002*randn(1,(N*fe));
n0 = [0; 0; 0; 0; 0; 0]; %noise vector
n = [n_ax; n_ay; n_az; n_wx; n_wy; n_wz]; %noise vector

di_t = di;
di_t0 = di0;

%FORMULATION IS PQV
%UNIT QUATERNION IS [1 0 0 0]

for i=1:N*fe-1
    d = data2delta(b0, u(:,i), n0, dt);
%% test imu_integrator

    di_out0 = imu_integrator(di, d, dt);
    di=di_out0;
    di_t = [di_t, di];
end

%% plot orientation over time
 qr = 4:7;
 orientation = [];
 angle_reconstruct = [];

for j=1:size(di_t,2)
    q =  di_t(qr,j);
    pos = q2R(q)*init_pos;
    orientation = [orientation pos];
    angle_reconstruct = [angle_reconstruct q2v(q)];
end

figure('Name','3D orientation plot - preintegrated','NumberTitle','off');
plot3(orientation(1,:), orientation(2,:), orientation(3,:), 'g');
hold on;
plot3(total_pos(1,:), total_pos(2,:), total_pos(3,:),'r');
plot3(0,0,0,'r*');
plot3(1,0,0,'g*');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('reconstructed orientation state', 'real orientation state');

% error
error = total_pos - orientation;

figure('Name','3D orientation error plot','NumberTitle','off');;
plot3(error(1,:), error(2,:), error(3,:));
hold on;
plot3(0,0,0,'r*');
plot3(1,0,0,'g*');
xlabel('x error');
ylabel('y error');
zlabel('z error');

figure('Name','orientation error through time','NumberTitle','off');
subplot(3,1,1);
plot(t, error(1,:));
xlabel('time');
ylabel('error x');
hold on;
subplot(3,1,2);
plot(t, error(2,:));
xlabel('time');
ylabel('error y');
subplot(3,1,3);
plot(t, error(3,:));
xlabel('time');
ylabel('error z');

figure('Name','orientation through time','NumberTitle','off');
subplot(3,1,1);
plot(t, angle_reconstruct(1,:));
hold on;
plot(t, ox(1,:),'r');
xlabel('time');
ylabel('angle x');
hold on;
subplot(3,1,2);
plot(t, angle_reconstruct(2,:));
hold on;
plot(t, oy(1,:),'r');
xlabel('time');
ylabel('angle y');
subplot(3,1,3);
plot(t, angle_reconstruct(3,:));
hold on;
plot(t, oz(1,:),'r');
xlabel('time');
ylabel('angle z');