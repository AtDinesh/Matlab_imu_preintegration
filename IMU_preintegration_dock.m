%%
% generate a trajectory using cos and/or sin. Derive twice to get acceleration
% and give those value to the imu integrator to recompute the trajectory.
% Using trigonometry is ok : position won't go to infinity + loop.

%*****************************WORKING*********************

addpath('RandomWalk');
%%
%close all;
clear all;

write_to_file_const = true;

fe = 1000;
dt = 1/fe;
N = 1;

% generateIMU(x0, x1, hx, y0, y1, hy, z0, z1, hz, ox0, ox1, ohx, oy0, oy1, ohy, oz0, oz1, ohz, t0, t1, dt )
x0=0; x1=0; hx=0.2; y0=0; y1=-0.06; hy=0; z0=0; z1=0; hz=0.1; 
ox0=0; ox1=0; ohx=0.6; oy0=0; oy1=0; ohy=0.3; oz0=0; oz1=0; ohz=0.4; 
t0=0; t1=1;

[a w t] = generateIMU(x0, x1, hx, y0, y1, hy, z0, z1, hz, ox0, ox1, ohx, oy0, oy1, ohy, oz0, oz1, ohz, t0, t1, dt );

static_time = 1; %s

vec03 = [0; 0; 0];
for i = 1:1:fe*static_time
    a_final(1:3,i) = vec03;
    w_final(1:3,i) = vec03;
    t_final(1,i) = i*dt;
end

a_final(1:3,1001:1001+(size(a,2)-2)) = a(1:3,2:1001);
w_final(1:3,1001:1001+(size(a,2)-2)) = w(1:3,2:1001);
t_final(1,1001:1001+(size(t,2)-2)) = t(1,2:1001)+t_final(1,1000);

for i = size(t_final,2):1:fe*static_time+size(t_final,2)
    a_final(1:3,i) = vec03;
    w_final(1:3,i) = vec03;
    t_final(1,i) = i*dt;
end

a = a_final;
w = w_final;
t = t_final;

a0 = [0;0;9.806];
u = [a;w];
%% needed parameters

di = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
initial_condition = di;
di0 = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
di1 = di0;

b = [0.1; 0.12; -0.1; 0.015; -0.027; 0.01]; %bias vector
%b = [0; 0; 0; 0; 0; 0];
b0 = [0; 0; 0; 0; 0; 0];

n_ax = 0.053*randn(1,size(u,2)); %0.012
n_ay = 0.053*randn(1,size(u,2));
n_az = 0.053*randn(1,size(u,2));
n_wx = 0.001*randn(1,size(u,2)); %0.15
n_wy = 0.001*randn(1,size(u,2));
n_wz = 0.001*randn(1,size(u,2));
n0 = [0; 0; 0; 0; 0; 0]; %noise vector
n = [n_ax; n_ay; n_az; n_wx; n_wy; n_wz]; %noise vector

n_bax = 0.002*randn(1,size(u,2)); %0.012
n_bay = 0.001*randn(1,size(u,2));
n_baz = 0.0015*randn(1,size(u,2));
n_bwx = 0.0007*randn(1,size(u,2)); %0.15
n_bwy = 0.0005*randn(1,size(u,2));
n_bwz = 0.0008*randn(1,size(u,2));
nb = [n_bax; n_bay; n_baz; n_bwx; n_bwy; n_bwz]; %noise vector
r_walk = cumsum(nb);

% One dimension random walk
N = 1; % Dimensions
M = t(1,numel(t))*fe;
bAx = zeros(M,1); bAy = zeros(M,1); bAz = zeros(M,1);
bWx = zeros(M,1); bWy = zeros(M,1); bWz = zeros(M,1);
%%
% We now simulate t*fe steps of a random walk
for ii = 2:M
    bAx(ii) = bAx(ii-1)+RandDir(N)*0.005;
    bAy(ii) = bAy(ii-1)+RandDir(N)*0.005;
    bAz(ii) = bAz(ii-1)+RandDir(N)*0.005;
    bWx(ii) = bWx(ii-1)+RandDir(N)*0.005;
    bWy(ii) = bWy(ii-1)+RandDir(N)*0.005;
    bWz(ii) = bWz(ii-1)+RandDir(N)*0.005;
end

figure();
subplot(3,2,1);
comet(1:numel(bAx),bAx);
plot(1:numel(bAx),bAx);
title('random walk ax bias');
grid on;

subplot(3,2,3);
comet(1:numel(bAy),bAy);
plot(1:numel(bAy),bAy);
title('random walk ay bias');
grid on;

subplot(3,2,5);
comet(1:numel(bAz),bAz);
plot(1:numel(bAz),bAz);
title('random walk az bias');
grid on;

subplot(3,2,2);
comet(1:numel(bWx),bWx);
plot(1:numel(bWx),bWx);
title('random walk wx bias');
grid on;

subplot(3,2,4);
comet(1:numel(bWy),bWy);
plot(1:numel(bWy),bWy);
title('random walk wy bias');
grid on;

subplot(3,2,6);
comet(1:numel(bWz),bWz);
plot(1:numel(bWz),bWz);
title('random walk wz bias');
grid on;

r_walk = [bAx'; bAy'; bAz'; bWx'; bWy'; bWz'];

di_t = [];
di_t0 = di0;
state_vec = di;
state_n_vec = di;
state = [];
state_n = [];
di_out1 = [];
ur_vec = [];

%FORMULATION IS PQV
%UNIT QUATERNION IS [1 0 0 0]

for i=1:size(u,2)
    %current_orientation = q2v(state_vec(4:7, size(state_vec,2)));
    %R0_1 = v2R(current_orientation);
    R0_1 = q2R(state_vec(4:7, size(state_vec,2)));
    aR = inv(R0_1) * a0;
    
    %% proceed without noise to get odometry
    ur(1:3,1) = inv(R0_1) * u(1:3,i) + aR;
    ur(4:6,1) = u(4:6,i);
    ur_vec(1:6,i) = ur;

    d = data2delta(b0, ur(:,1), n0, dt);
    di_out0 = imu_integrator(di0, d, dt);

    di0 = di_out0;
    di_out1 = [di_out1 di_out0];
    
    % state reconstruction for odometry
    Dt = t(1,i);
    state = xPlusDelta(initial_condition, di_out0, Dt);
    state_vec(1:10,i) =  state;
    
    %% proceed with noise to simulate a noisy IMU
    
    % express the acceleration in the global frame if we want the IMU to
    % follow the trajectory defined by [x y z]. Then the IMU will rotate
    % along this trajectory and the accelerations will be expressed in the
    % new local frame
    
    %u1(1:3,i) = u(1:3,i) + aR;
    ur_noisy(1:3,1) = inv(R0_1) * u(1:3,i) + aR; %inv(R0_1) * u(1:3,i) + aR;
    ur_noisy(4:6,1) = u(4:6,i);
    
    % add bias and noise
    ur_noisy(:,1) = ur_noisy(:,1) + (b + r_walk(:,i)) + n(:,i);
    %ur_noisy(:,1) = ur_noisy(:,1) + b;
    %ur_noisy(:,1) = ur_noisy(:,1) + n(:,i);
    ur_noisy_vec(1:6,i) = ur_noisy;

    d = data2delta(b0, ur_noisy(:,1), n0, dt);
    di_out0 = imu_integrator(di1, d, dt);

    di1=di_out0;
    di_t = [di_t, di1];
    state_n = xPlusDelta(initial_condition, di_out0, Dt);
    state_n_vec(1:10,i) =  state_n;
end

DT = Dt; 
x_final = xPlusDelta(initial_condition, di0, DT);


%% all 3D plots in same figure :
figure('Name','compare trajectories','NumberTitle','off');
plot3(state_n_vec(1,:),state_n_vec(2,:),state_n_vec(3,:), 'g');
hold on;
plot3(state_vec(1,:), state_vec(2,:), state_vec(3,:), 'm');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('integrated trajectory', 'integrated states');

%% time plot
figure('Name','plot wrt time','NumberTitle','off');
subplot(2,2,1);
plot(t(1,:), state_n_vec(1,:), 'r');
hold on;
plot(t(1,:), state_n_vec(2,:), 'g');
plot(t(1,:), state_n_vec(3,:), 'b');
xlabel('time');
ylabel('position (m)');
legend('x trajectory', 'y trajectory', 'z trajectory');

subplot(2,2,2);
plot(t(1,:), state_n_vec(8,:), 'r');
hold on;
plot(t(1,:), state_n_vec(9,:), 'g');
plot(t(1,:), state_n_vec(10,:), 'b');
xlabel('time');
ylabel('velocity (m/s)');
legend('vx', 'vy', 'vz');

subplot(2,2,3);
plot(t(1,:), state_vec(1,:), 'r');
hold on;
plot(t(1,:), state_vec(2,:), 'g');
plot(t(1,:), state_vec(3,:), 'b');
xlabel('time');
ylabel('position (m)');
legend('x unbiased trajectory', 'y unbiased trajectory', 'z unbiased trajectory');

subplot(2,2,4);
plot(t(1,:), state_vec(8,:), 'r');
hold on;
plot(t(1,:), state_vec(9,:), 'g');
plot(t(1,:), state_vec(10,:), 'b');
xlabel('time');
ylabel('velocity (m/s)');
legend('vx unbiased', 'vy unbiased', 'vz unbiased');

%% write data in file

if(write_to_file_const)
    fileID = fopen('simulated_imuData.txt','wt');
    fprintf(fileID,'%3.12f\t',initial_condition'); %initial condition is the first line of the file
    fprintf(fileID,'%3.12f\t',b'); %bias
    fprintf(fileID,'\n');
    fprintf(fileID,'%3.12f\t',x_final);
    fprintf(fileID,'\n');
    data = [t',ur_noisy_vec'];
    for ii = 1:size(data,1)
        fprintf(fileID,'%3.12f\t',data(ii,:));
        fprintf(fileID,'\n');
    end
    fclose(fileID);
    
    fileID_check = fopen('simulated_odomData.txt','wt');
    step = 1000;
    t_odom = [];
    p_odom = [];
    o_odom = [];
    for iter = step:step:size(u,2)
        t_odom = [t_odom, t(:,iter)];
        p_odom = [p_odom, qRot(state_vec(1:3,iter) -  state_vec(1:3,iter - step+1), q2qc(state_vec(4:7, iter-step+1)))];
        o_odom = [o_odom, q2v(qProd(q2qc(state_vec(4:7, iter-step+1)), state_vec(4:7, iter))) ];
    end
    odom = [t_odom', p_odom',o_odom'];
    for ii = 1:size(odom,1)
        fprintf(fileID_check,'%3.12f\t',odom(ii,:));
        fprintf(fileID_check,'\n');
    end
    fclose(fileID_check);
end