%% 
% Testing rotations + translation.
% Generate a rotation angle on x, y and z axes (ox, oy, oz in radians). These angles change over 
% time. -> we comput the rate of turn components wx, wy and wz in global
% frame.

% Interest point also moves in time following sinus function. We
% compute acceleration components ax, ay and az in global frame. To feed
% the imu integrator we have to express those components in the local frame
% of IMU.

% We suppose that IMU local frame initially matches with global frame. This
% imu is moved and rotated over time. We can deduce the rotation of the imu
% from the Deltas (that we get from the IMU integrator). Let Q be this
% quaternion. Then we deduce R the corresponding rotation matrix, G the
% global frame and L the local frame. Then in term of orientation, we have
% L = R*G. We can express the acceleration vector in the IMU local frame
% doing : acc_L = inv(R) * acc_G
% similar reasoning for rate of turn initially expressed in global frame,
% we have : w_L = inv(R) * w_G

%*****************************SEEMS TO BE WORKING*********************

%% prepare
close all;
clear all;

fe = 1000;
N = 100*1;
t = (0:1/fe:N-1/fe);

write_to_file = false;

%% position + orientation
%position
x = sin(t);
y = sin(2*t);
z = sin(2*t);

%orientation
alpha = 50; %degree per second rotation around x axis
beta = 70; %degree per second rotation around y axis
gamma = 100; %degree per second rotation around z axis

ox = alpha*t*pi/180;
oy = beta*t*pi/180;
oz = gamma*t*pi/180;

%% construct data in R0
deg_to_rad = 3.14159265359/180.0;
ax = -sin(t);
ay = -4*sin(2*t);
az = -4*sin(2*t);
wx(1,1:(N*fe)) = alpha*pi/180;
wy(1,1:(N*fe)) = beta*pi/180;
wz(1,1:(N*fe)) = gamma*pi/180;
u = [ax; ay; az; wx; wy; wz];

%% needed parameters

dt = 1/fe;
di = [0; 0; 0; 1; 0; 0; 0; 1; 2; 2];
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

total_pos = [];
v = [ox(1,:); oy(1,:); oz(1,:)]; %this is the evolution of rotations
R0_1 = [1 0 0; 0 1 0; 0 0 1];
u1 = [];

%% test

angle_reconstruct = [];
u1 = u; 
local_x = [1;0;0];
local_y = [0;1;0];
local_z = [0;0;1];

for i=1:N*fe-1
    %change to local coordinate system
    u1(1:3,i) = inv(R0_1) * u(1:3,i);
    u1(4:6,i) = inv(R0_1) * u(4:6,i);
    d = data2delta(b0, u1(:,i), n0, dt);
%% test imu_integrator

    di_out0 = imu_integrator(di, d, dt);
    di=di_out0;
    R0_1 = q2R(di_out0(4:7)); %update the rotation matrix to pass from global to local
    di_t = [di_t, di];
end

%% ANALYZE             ******************* PLOTTING PART *******************

%%  plot orientation over time
 qr = 4:7;
 angle_reconstruct = [];

for j=1:size(di_t,2)
    q =  di_t(qr,j);
    angle_reconstruct = [angle_reconstruct q2v(q)];
end

figure('Name','orientation through time','NumberTitle','off');
subplot(3,2,1);
plot(t, angle_reconstruct(1,:));
hold on;
plot(t, ox(1,:),'r');
xlabel('time');
ylabel('angle x (rad)');
legend('real orientation state', 'reconstructed orientation state');
subplot(3,2,2);
plot(t, ox(1,:) - angle_reconstruct(1,:));
xlabel('time');
ylabel('angle x - ERROR (rad)');
subplot(3,2,3);
plot(t, angle_reconstruct(2,:));
hold on;
plot(t, oy(1,:),'r');
xlabel('time');
ylabel('angle y (rad)');
legend('real orientation state', 'reconstructed orientation state');
subplot(3,2,4);
plot(t, oy(1,:) - angle_reconstruct(2,:));
xlabel('time');
ylabel('angle y - ERROR (rad)');
subplot(3,2,5);
plot(t, angle_reconstruct(3,:));
hold on;
plot(t, oz(1,:),'r');
xlabel('time');
ylabel('angle z (rad)');
legend('real orientation state', 'reconstructed orientation state');
subplot(3,2,6);
plot(t, oz(1,:) - angle_reconstruct(3,:));
xlabel('time');
ylabel('angle z - ERROR (rad)');

%% plot Position
figure('Name','3D position plot','NumberTitle','off');
plot3(x,y,z, 'g');
hold on;
plot3(di_t(1,:), di_t(2,:), di_t(3,:), 'r');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('real position', 'reconstructed position');

figure('Name','position error','NumberTitle','off')
subplot(3,1,1);
plot(t,x(1,:) - di_t(1,:));
xlabel('time');
ylabel('x error over time');
hold on;
subplot(3,1,2);
plot(t,y(1,:) - di_t(2,:));
xlabel('time');
ylabel('y error over time');
subplot(3,1,3);
plot(t,z(1,:) - di_t(3,:));
xlabel('time');
ylabel('z error over time');


%% checking acceleration

figure('Name','compare acceleration in local and global frames','NumberTitle','off')
subplot(3,1,1);
plot(t,u1(1,:), 'r');
hold on;
plot(t,u(1,:), 'g');
xlabel('time');
ylabel('x accel over time');
legend('ax in local frame', 'ax in global frame');
hold on;
subplot(3,1,2);
plot(t,u1(2,:), 'r');
hold on;
plot(t,u(2,:), 'g');
xlabel('time');
ylabel('y accel over time');
legend('ax in local frame', 'ax in global frame');
subplot(3,1,3);
plot(t,u1(3,:), 'r');
hold on;
plot(t,u(3,:), 'g');
xlabel('time');
ylabel('z accel over time');
legend('ax in local frame', 'ax in global frame');


%% write in file

if(write_to_file)
    fileID = fopen('data_final_test_az.txt','wt');
    data = [t',u1'];
    for ii = 1:size(data,1)
        fprintf(fileID,'%g\t',data(ii,:));
        fprintf(fileID,'\n');
    end
    fclose(fileID);
    
    fileID_check = fopen('data_final_test_checkPosition_az.txt','wt');
    data = [t',x',y',z'];
    for ii = 1:size(data,1)
        fprintf(fileID_check,'%g\t',data(ii,:));
        fprintf(fileID_check,'\n');
    end
    fclose(fileID_check);
end

