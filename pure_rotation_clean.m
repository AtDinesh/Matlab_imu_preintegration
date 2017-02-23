%% 
% Testing rotations.
% Generate a rotation angle on x, y and z axes (ox, oy, oz in radians). These angles change over 
% time.
% 
% From formulas for ox, oy and oz, get wx, wy, wz that matches the rate of turn we will give to IMU data integrator.
% Compute data using the imu_integrator with ax=ay=az=0 (pure rotation here).

%% 
clear all;
close all;

write_to_file_const = true;
write_to_file = false;

fe = 1000;
N = 1;
t = (0:1/fe:N);

alpha = 5;
beta = 2;
gamma = 5;

init_pos = [1; 0; 0];

%ox oy oz evolution in degrees (for understanding) --> converted in rad
%with * pi/180
ox = pi*sin(alpha*t*pi/180); %express angle in rad before using sinus
oy = pi*sin(beta*t*pi/180);
oz = pi*sin(gamma*t*pi/180);

x = 0*t;
y = 0*t;
z = 0*t;

ax = 0*t;
ay = 0*t;
az = 0*t;
a0 = [0; 0; 9.8];

%rate of turn expressed in radians/s
wx = pi*alpha*cos(alpha*t*pi/180)*pi/180;
wy = pi*beta*cos(beta*t*pi/180)*pi/180;
wz = pi*gamma*cos(gamma*t*pi/180)*pi/180;

u = [ax; ay; az; wx; wy; wz];
%% needed parameters

dt = 1/fe;
di = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
initial_condition = di;
di0 = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];

b0 = [0; 0; 0; 0; 0; 0]; %bias vector

n_ax = 0.04*randn(1,(N*fe));
n_ay = 0.04*randn(1,(N*fe));
n_az = 0.04*randn(1,(N*fe));
n_wx = 0.002*randn(1,(N*fe));
n_wy = 0.002*randn(1,(N*fe));
n_wz = 0.002*randn(1,(N*fe));
n0 = [0; 0; 0; 0; 0; 0]; % Zero noise vector
n = [n_ax; n_ay; n_az; n_wx; n_wy; n_wz]; %noise vector

di_t = [];
di_t0 = di0;
state_vec = di;
state = [];

%FORMULATION IS PQV
%UNIT QUATERNION IS [1 0 0 0]

% We know how the angle varies --> i.e. we have the rate of turn
% components. The integrator deduces how the angle should vary from current
% state to next state. The angle information is stored in quaternion
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

%% plot orientation over time
 orientation = [];
 Dq = [];
for j=1:size(di_t,2)
    q =  di_t(4:7,j);
    R0_1 = q2R(q);
    pos = inv(R0_1) * init_pos;
    orientation = [orientation pos];
    Dq = [Dq q];
end



%% check quaternions
figure('Name','compare quaternions','NumberTitle','off')
subplot(2,2,1);
plot(t, Dq(2,:),'r');
xlabel('time');
ylabel('qx');
legend('estimated qx');

subplot(2,2,2);
plot(t, Dq(3,:),'r');
xlabel('time');
ylabel('qy');
legend('estimated qy');

subplot(2,2,3);
plot(t, Dq(4,:),'r');
xlabel('time');
ylabel('qz');
legend('estimated qz');


subplot(2,2,4);
plot(t, Dq(1,:),'r');
xlabel('time');
ylabel('qw');
legend('estimated qw');

%% plot3D
figure('Name','orientation plot','NumberTitle','off')
plot3(orientation(1,:), orientation(2,:), orientation(3,:), 'r');
xlabel('x');
ylabel('y');
zlabel('z');


%% write in file

if(write_to_file_const)
    fileID = fopen('data_pure_rotation.txt','wt');
    fprintf(fileID,'%3.16f\t',initial_condition'); %initial condition is the first line of the file
    fprintf(fileID,'\n');
    data = [t',u1'];
    for ii = 1:size(data,1)
        fprintf(fileID,'%3.16f\t',data(ii,:));
        fprintf(fileID,'\n');
    end
    fclose(fileID);
    
    fileID_check = fopen('odom_pure_rotation.txt','wt');
    fprintf(fileID_check,'%3.16f\t',x_final);
    fprintf(fileID_check,'\n');
    step = 10;
    step_up = step+1;
    t_odom = [];
    p_odom = [];
    o_odom = [];
    for iter = step_up:step:size(x,2)
        t_odom = [t_odom, t(:,iter)];
        p_odom = [p_odom, state_vec(1:3,iter) -  state_vec(1:3,iter - step)];
        o_odom = [o_odom, q2v(qProd(q2qc(state_vec(4:7, iter-step)), state_vec(4:7, iter))) ];
        %o_odom = [o_odom, [0; 0; 0]];
    end
    odom = [t_odom', p_odom',o_odom'];
    for ii = 1:size(odom,1)
        fprintf(fileID_check,'%3.16f\t',odom(ii,:));
        fprintf(fileID_check,'\n');
    end
    fclose(fileID_check);
end