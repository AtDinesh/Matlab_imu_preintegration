%%
% generate a trajectory using cos and/or sin. Derive twice to get acceleration
% and give those value to the imu integrator to recompute the trajectory.
% Using trigonometry is ok : position won't go to infinity + loop.

%THIS IS PURE TRANLSATION
%rate of turn set to 0 (wx = wy= wz = 0) for all the trajectory.

%*****************************WORKING*********************
%%
close all;
clear all;

write_to_file_const = false;
write_to_file = false;

fe = 1000;
N = 10*1;
t = (0:1/fe:(N-1/fe)+1/fe);

x = sin(t);
y = sin(2*t);
z = sin(2*t);

p = [x; y; z];

figure('Name','3D position plot','NumberTitle','off');
plot3(x,y,z);
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('generated trajectory');

deg_to_rad = pi/180.0;
ax = -sin(t);
ay = -4*sin(2*t);
az = -4*sin(2*t);
a0 = [0; 0; 9.806];
wx(1,1:(N*fe)+1) = 0*deg_to_rad; %+1 to get the data at timestamp 10s in IMU + odom
wy(1,1:(N*fe)+1) = 0*deg_to_rad;
wz(1,1:(N*fe)+1) = 0*deg_to_rad;
u = [ax; ay; az; wx; wy; wz];

ox = 0*t;
oy = 0*t;
oz = 0*t;

o = [ox; oy; oz];

%% needed parameters

dt = 0.001;
di = [0; 0; 0; 1; 0; 0; 0; 1.0; 2.0; 2.0];
initial_condition = di;
di0 = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];

b0 = [0; 0; 0; 0; 0; 0]; %bias vector

n_ax = 0.04*randn(1,(N*fe));
n_ay = 0.04*randn(1,(N*fe));
n_az = 0.04*randn(1,(N*fe));
n_wx = 0.002*randn(1,(N*fe));
n_wy = 0.002*randn(1,(N*fe));
n_wz = 0.002*randn(1,(N*fe));
n0 = [0; 0; 0; 0; 0; 0]; %noise vector
n = [n_ax; n_ay; n_az; n_wx; n_wy; n_wz]; %noise vector

di_t = [];
di_t0 = di0;

%FORMULATION IS PQV
%UNIT QUATERNION IS [1 0 0 0]

for i=1:N*fe+1
    R0_1 = v2R(o(:,i));
    aR = inv(R0_1) * a0;
    %u(1:3,i) = u(1:3,i) + aR;
    u1(1:3,i) = inv(R0_1) * u(1:3,i) + aR;
    u1(4:6,i) = u(4:6,i);
    d = data2delta(b0, u1(:,i), n0, dt);
%% test imu_integrator

    di_out0 = imu_integrator(di, d, dt);
    di=di_out0;
    di_t = [di_t, di];
end

Dt = t(size(t,2))- t(1);
x_final = xPlusDelta(initial_condition, di_out0, Dt);

%% Plot Integrated position
figure('Name','Acceleration, Velocity and Position','NumberTitle','off');
subplot(3,3,1);
plot(t, ax(1,:));
hold on;
xlabel('time');
ylabel('Acceleration X');
subplot(3,3,4);
plot(t, ay(1,:));
xlabel('time');
ylabel('Acceleration Y');
subplot(3,3,7);
plot(t, az(1,:));
xlabel('time');
ylabel('Acceleration Z');

%figure('Name','velocity through time','NumberTitle','off');
subplot(3,3,2);
plot(t, di_t(8,:));
hold on;
xlabel('time');
ylabel('Velocity X');
subplot(3,3,5);
plot(t, di_t(9,:));
xlabel('time');
ylabel('Velocity Y');
subplot(3,3,8);
plot(t, di_t(10,:));
xlabel('time');
ylabel('Velocity Z');

%figure('Name','position through time','NumberTitle','off');
subplot(3,3,3);
plot(t, x(1,:));
xlabel('time');
ylabel('Position X');
hold on;
subplot(3,3,6);
plot(t, y(1,:));
xlabel('time');
ylabel('Position Y');
subplot(3,3,9);
plot(t, z(1,:));
xlabel('time');
ylabel('Position Z');

%% 3D plot
% figure('Name','3D position plot','NumberTitle','off');
% plot3(di_t(1,:),di_t(2,:),di_t(3,:));
% xlabel('x posititon');
% ylabel('y posititon');
% zlabel('z posititon');

% fileID = fopen('data_test_trig.txt','wt');
% data = [t',u'];
% for ii = 1:size(data,1)
%     fprintf(fileID,'%g\t',data(ii,:));
%     fprintf(fileID,'\n');
% end
% fclose(fileID);
% 
% figure('Name','2D position plot','NumberTitle','off');
% subplot(3,2,1)
% plot(x,y);
% xlabel('x posititon');
% ylabel('y posititon');
% subplot(3,2,2)
% plot(di_t(1,:),di_t(2,:));
% xlabel('x posititon integrated');
% ylabel('y posititon integrated');
% subplot(3,2,3)
% plot(x,z);
% xlabel('x posititon');
% ylabel('z posititon');
% subplot(3,2,4)
% plot(di_t(1,:),di_t(3,:));
% xlabel('x posititon integrated');
% ylabel('z posititon integrated');
% subplot(3,2,5)
% plot(y,z);
% xlabel('y posititon');
% ylabel('z posititon');
% subplot(3,2,6)
% plot(di_t(2,:),di_t(3,:));
% xlabel('y posititon integrated');
% ylabel('z posititon integrated');

figure('Name','error','NumberTitle','off')
subplot(3,1,1);
plot(t,x(1,:) - di_t(1,:));
xlabel('x error over time');
hold on;
subplot(3,1,2);
plot(t,y(1,:) - di_t(2,:));
xlabel('y error over time');
subplot(3,1,3);
plot(t,z(1,:) - di_t(3,:));
xlabel('z error over time');


%% all 3D plots in same figure :
figure('Name','compare trajectories','NumberTitle','off');
plot3(x(1,:),y(1,:),z(1,:), 'r');
hold on;
plot3(di_t(1,:),di_t(2,:),di_t(3,:), 'g');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('real trajectory', 'integrated trajectory');


%% write data in file

if(write_to_file)
    fileID = fopen('data_pure_translation.txt','wt');
    data = [t',u'];
    for ii = 1:size(data,1)
        fprintf(fileID,'%g\t',data(ii,:));
        fprintf(fileID,'\n');
    end
    fclose(fileID);
    
    fileID_check = fopen('data_pure_translation_check.txt','wt');
    data = [t',x',y',z', o'];
    for ii = 1:size(data,1)
        fprintf(fileID_check,'%g\t',data(ii,:));
        fprintf(fileID_check,'\n');
    end
    fclose(fileID_check);
end


if(write_to_file_const)
    fileID = fopen('data_pure_translation.txt','wt');
    fprintf(fileID,'%g\t',initial_condition'); %initial condition is the first line of the file
    fprintf(fileID,'\n');
    data = [t',u1'];
    for ii = 1:size(data,1)
        fprintf(fileID,'%g\t',data(ii,:));
        fprintf(fileID,'\n');
    end
    fclose(fileID);
    
    fileID_check = fopen('odom_pure_translation.txt','wt');
    step = 5;
    step_up = step+1;
    t_odom = [];
    p_odom = [];
    o_odom = [];
    for iter = step_up:step:size(x,2)
        t_odom = [t_odom, t(:,iter)];
        p_odom = [p_odom, inv(v2R(o(:,iter) - o(:,iter - step))) * p(:,iter) - p(:,iter - step)];
        o_odom = [o_odom, o(:,iter) - o(:,iter - step)];
    end
    odom = [t_odom', p_odom',o_odom'];
    for ii = 1:size(odom,1)
        fprintf(fileID_check,'%g\t',odom(ii,:));
        fprintf(fileID_check,'\n');
    end
    fclose(fileID_check);
end