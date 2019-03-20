%%
% generate a trajectory using cos and/or sin. Derive twice to get acceleration
% and give those value to the imu integrator to recompute the trajectory.
% Using trigonometry is ok : position won't go to infinity + loop.

%*****************************WORKING*********************
%%
close all;
clear all;

write_to_file_const = false;
write_to_file = false;

fe = 1000;
dt = 1/fe;
N = 1;
t = (0:1/fe:N);
t3 = (0:1/fe:3*N + 3/fe);

x = 0*t;
y = 0*t;
z = 0*t;

ax = 0*t;
ay = 0*t;
az = 0*t;
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

Ovec = 0*t;

u = [ax; ay; az; Ovec; Ovec; Ovec];
%% needed parameters

di = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
initial_condition = di;
di0 = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
di1 = di0;

b = [0.06; 0.025; -0.0033; 0.045; -0.027; 0.08]; %bias vector
b0 = [0; 0; 0; 0; 0; 0];

n_ax = 0.012*randn(1,size(u,2));
n_ay = 0.012*randn(1,size(u,2));
n_az = 0.012*randn(1,size(u,2));
n_wx = 0.15*randn(1,size(u,2));
n_wy = 0.15*randn(1,size(u,2));
n_wz = 0.15*randn(1,size(u,2));
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
    else
        d = data2delta(b0, ur(:,1), n0, 0);
    end
    
    if (i ~= 1)
        di_out0 = imu_integrator(di0, d, dt);
    else
        di_out0 = imu_integrator(di0, d, 0);
    end
    
    di0 = di_out0;
    di_out1 = [di_out1 di_out0];
    
    % state reconstruction for odometry
    Dt = t3(1,i);
    state = xPlusDelta(initial_condition, di_out0, Dt);
    state_vec(1:10,i) =  state;
end

DT = t3(size(t3,2))- t3(1) ;
x_final = xPlusDelta(initial_condition, di_out0, DT);


%% all 3D plots in same figure :
figure('Name','compare trajectories','NumberTitle','off');
plot3(x(1,:),y(1,:),z(1,:), 'r');
hold on;
plot3(state_vec(1,:), state_vec(2,:), state_vec(3,:), 'm');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('real trajectory', 'integrated states');


%% write data in file

if(write_to_file_const)
    fileID = fopen('data_trajectory_full.txt','wt');
    fprintf(fileID,'%3.16f\t',initial_condition'); %initial condition is the first line of the file
    fprintf(fileID,'%3.16f\t',b0'); %bias
    fprintf(fileID,'\n');
    data = [t',u1'];
    for ii = 1:size(data,1)
        fprintf(fileID,'%3.16f\t',data(ii,:));
        fprintf(fileID,'\n');
    end
    fclose(fileID);
    
    fileID_check = fopen('odom_trajectory_full.txt','wt');
    fprintf(fileID_check,'%3.16f\t',x_final);
    fprintf(fileID_check,'\n');
    step = 1000;
    step_up = step+1;
    t_odom = [];
    p_odom = [];
    o_odom = [];
    for iter = step_up:step:size(x,2)
        t_odom = [t_odom, t(:,iter)];
        p_odom = [p_odom, state_vec(1:3,iter) -  state_vec(1:3,iter - step)];
        o_odom = [o_odom, q2v(qProd(q2qc(state_vec(4:7, iter-step)), state_vec(4:7, iter))) ];
    end
    odom = [t_odom', p_odom',o_odom'];
    for ii = 1:size(odom,1)
        fprintf(fileID_check,'%3.16f\t',odom(ii,:));
        fprintf(fileID_check,'\n');
    end
    fclose(fileID_check);
end