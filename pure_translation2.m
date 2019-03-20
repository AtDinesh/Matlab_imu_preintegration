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

write_to_file_const = true;
write_to_file = false;

fe = 1000;
N = 1;
t = (0:1/fe:N);

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
a0 = [0; 0; 9.8];
wx(1,1:(N*fe)+1) = 0*deg_to_rad; %+1 to get the data at timestamp 10s in IMU + odom
wy(1,1:(N*fe)+1) = 0*deg_to_rad;
wz(1,1:(N*fe)+1) = 0*deg_to_rad;
u = [ax; ay; az; wx; wy; wz];

ox = 0*t;
oy = 0*t;
oz = 0*t;

o = [ox; oy; oz];

%% needed parameters

dt = 1/fe;
di = [0; 0; 0; 1; 0; 0; 0; 1; 2.0; 2.0];
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
state_vec = di;
state = [];

%FORMULATION IS PQV
%UNIT QUATERNION IS [1 0 0 0]

for i=1:N*fe+1
    current_orientation = q2v(state_vec(4:7, size(state_vec,2)));
    R0_1 = v2R(current_orientation);
    aR = inv(R0_1) * a0;
    u1(1:3,i) = u(1:3,i) + aR;
    %u1(1:3,i) = inv(R0_1) * u(1:3,i) + aR;
    u1(4:6,i) = u(4:6,i);
    
    if(i ~= 1)
        d = data2delta(b0, u1(:,i), n0, dt);
    else
        d = data2delta(b0, u1(:,i), n0, 0);
    end
%% test imu_integrator
    
    if (i ~= 1)
        di_out0 = imu_integrator(di0, d, dt);
    else
        di_out0 = imu_integrator(di0, d, 0);
    end

    di0=di_out0;
    di_t = [di_t, di0];
    
    % state reconstruction
    Dt = t(1,i);
    state = xPlusDelta(initial_condition, di_out0, Dt);
    state_vec(1:10,i) =  state;
end

DT = t(size(t,2))- t(1) ;
x_final = xPlusDelta(initial_condition, di_out0, DT);

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


%% all 3D plots in same figure :
figure('Name','compare trajectories','NumberTitle','off');
plot3(x(1,:),y(1,:),z(1,:), 'r');
hold on;
plot3(di_t(1,:),di_t(2,:),di_t(3,:), 'g');
plot3(state_vec(1,:), state_vec(2,:), state_vec(3,:), 'm');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('real trajectory', 'integrated trajectory', 'integrated states');


%% write data in file

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
    fprintf(fileID_check,'%f\t',x_final);
    fprintf(fileID_check,'\n');
    step = 10;
    step_up = step+1;
    t_odom = [];
    p_odom = [];
    o_odom = [];
    for iter = step_up:step:size(x,2)
        t_odom = [t_odom, t(:,iter)];
        p_odom = [p_odom, state_vec(1:3,iter) -  state_vec(1:3,iter - step)];
        %p_odom = [p_odom, p(:,iter) -  p(:,iter - step)];
        %o_odom = [o_odom, q2v(qProd(q2qc(state_vec(4:7, iter-step)),state_vec(4:7, iter))) ];
        o_odom = [o_odom, [0; 0; 0]];
    end
    odom = [t_odom', p_odom',o_odom'];
    for ii = 1:size(odom,1)
        fprintf(fileID_check,'%g\t',odom(ii,:));
        fprintf(fileID_check,'\n');
    end
    fclose(fileID_check);
end