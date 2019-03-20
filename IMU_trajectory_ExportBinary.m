%%
% generate a trajectory using cos and/or sin. Derive twice to get acceleration
% and give those value to the imu integrator to recompute the trajectory.
% Using trigonometry is ok : position won't go to infinity + loop.

%*****************************WORKING*********************
%%
close all;
clear all;

write_to_file_const = true;

fe = 1000;
dt = 1/fe;
N = 1;
t = (0:1/fe:N);
t3 = (0:1/fe:2*N  + 1/fe);   % t3 = (0:1/fe:3*N + 2/fe);

x = 0*t3;
y = 0*t3;
z = 0*t3;

ax = 0*t + 0.01;
ay = 0*t + 0.04;
az = 0*t - 0.02;
a0 = [0; 0; 9.8];

%rate of turn expressed in radians/s
% we need a particular test case to be able to estimate the acceleration
% bias. We want the gravity to have an effect on all 3 axis but not at the
% same time.
% keep the imu static for 1 s -> integrate and create a KeyFrame
% rotate along x axis for 1 s -> integrate and create a keyFrame
% rotate along y axis for 1 s -> integrate and create a keyFrame
% after 1s integration we want the imu to come back to its original pose,
% that means the imu must perform a 2*pi rotation in 1s. Thus we do a
% rotation at constant rate of turn of 2*pi
wx(1:fix(size(t,2)/2)) = pi;
wx(fix(size(t,2)/2):size(t,2)) = 0;
wy(1:fix(size(t,2)/2)) = pi;
wy(fix(size(t,2)/2):size(t,2)) = 0;
Ovec = 0*t;

u0 = [Ovec; Ovec; Ovec; Ovec; Ovec; Ovec];
u1 = [ax; ay; az; Ovec; Ovec; Ovec];
u2 = [ax; ay; az; wx; Ovec; Ovec];
u3 = [ax; ay; az; Ovec; wy; Ovec];
uall = [ax; ay; az; wx; wy; Ovec];
uw = [Ovec; Ovec; Ovec; wx; wy; Ovec];

%u = [u0 u1 u2 u3 -u3 -u2 -u1];
u = [uw -uw];
%u = [u0 uw uw uw uw uw];
%% needed parameters

di = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
initial_condition = di;
di0 = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
di1 = di0;

b = [0.06; 0.025; -0.0033; 0.045; -0.027; 0.08]; %bias vector
b0 = [0; 0; 0; 0; 0; 0];

n_ax = 0.012*randn(1,size(u,2)); %0.012
n_ay = 0.012*randn(1,size(u,2));
n_az = 0.012*randn(1,size(u,2));
n_wx = 0.015*randn(1,size(u,2)); %0.15
n_wy = 0.015*randn(1,size(u,2));
n_wz = 0.015*randn(1,size(u,2));
n0 = [0; 0; 0; 0; 0; 0]; %noise vector
n = [n_ax; n_ay; n_az; n_wx; n_wy; n_wz]; %noise vector

di_t = [];
di_t0 = di0;
state_vec = di;
state = [];
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
    ur(1:3,1) = u(1:3,i) + aR;
    ur(4:6,1) = u(4:6,i);
    ur_vec(1:6,i) = ur;
    
    if(i ~= 1)
        d = data2delta(b0, ur(:,1), n0, dt);
        di_out0 = imu_integrator(di0, d, dt);
    else
        d = data2delta(b0, ur(:,1), n0, 0);
        di_out0 = imu_integrator(di0, d, 0);
    end
    
    di0 = di_out0;
    di_out1 = [di_out1 di_out0];
    
    % state reconstruction for odometry
    Dt = t3(1,i);
    state = xPlusDelta(initial_condition, di_out0, Dt);
    state_vec(1:10,i) =  state;
    
    %% proceed with noise to simulate a noisy IMU
    
    % express the acceleration in the global frame if we want the IMU to
    % follow the trajectory defined by [x y z]. Then the IMU will rotate
    % along this trajectory and the accelerations will be expressed in the
    % new local frame
    
    %u1(1:3,i) = u(1:3,i) + aR;
    ur_noisy(1:3,1) = u(1:3,i) + aR; %inv(R0_1) * u(1:3,i) + aR;
    ur_noisy(4:6,1) = u(4:6,i);
    
    % add bias and noise
    %ur_noisy(:,i) = ur_noisy(:,i) + b + n(:,i);
    ur_noisy(:,1) = ur_noisy(:,1) + b;
    ur_noisy_vec(1:6,i) = ur_noisy;
    
    if(i ~= 1)
        d = data2delta(b0, ur_noisy(:,1), n0, dt);
    else
        d = data2delta(b0, ur_noisy(:,1), n0, 0);
    end
    
    if (i ~= 1)
        di_out0 = imu_integrator(di1, d, dt);
    else
        di_out0 = imu_integrator(di1, d, 0);
    end

    di1=di_out0;
    di_t = [di_t, di1];
end

DT = Dt; %t3(size(t3,2))- t3(1) ;
x_final = xPlusDelta(initial_condition, di0, DT);


%% all 3D plots in same figure :
figure('Name','compare trajectories','NumberTitle','off');
plot3(di_t(1,:),di_t(2,:),di_t(3,:), 'g');
hold on;
plot3(state_vec(1,:), state_vec(2,:), state_vec(3,:), 'm');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('integrated trajectory', 'integrated states');


%% write data in file

if(write_to_file_const)
    fileID = fopen('simulation_testIMU_bin.txt','w');
    fwrite(fileID,initial_condition', 'double'); %initial condition is the first line of the file
    fwrite(fileID,b', 'double'); %bias
    fwrite(fileID,x_final, 'double');
    data = [t3',ur_noisy_vec'];
    for ii = 1:size(data,1)
        fwrite(fileID,data(ii,:), 'double');
    end
    fclose(fileID);
    
    fileID_check = fopen('simulation_testODOM_bin.txt','w');
    step = 5;
    step_up = step+1;
    t_odom = [];
    p_odom = [];
    o_odom = [];
    for iter = step_up:step:size(x,2)
        t_odom = [t_odom, t3(:,iter)];
        p_odom = [p_odom, qRot(state_vec(1:3,iter) -  state_vec(1:3,iter - step), q2qc(state_vec(4:7, iter-step)))];
        o_odom = [o_odom, q2v(qProd(q2qc(state_vec(4:7, iter-step)), state_vec(4:7, iter))) ];
    end
    odom = [t_odom', p_odom',o_odom'];
    for ii = 1:size(odom,1)
        fwrite(fileID_check,odom(ii,:), 'double');
    end
    fclose(fileID_check);
end