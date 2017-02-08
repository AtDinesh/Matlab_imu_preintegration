%% 
% Testing rotations.
% Generate a rotation angle on x, y and z axes (ox, oy, oz in radians). These angles change over 
% time.
%
% all 3D visualization deleted.
% 
% From formulas for ox, oy and oz, get wx, wy, wz that matches the rate of turn we will give to IMU data integrator.
% Compute data using the imu_integrator with ax=ay=az=0 (pure rotation here).

%***************************** WORKING *********************

%% 
clear all;
close all;

write_to_file_const = true;
write_to_file = false;

fe = 1000;
N = 10*1;
t = (0:1/fe:N);

alpha = 5;
beta = 2;
gamma = 5;

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
a0 = [0; 0; 9.806];

%rate of turn expressed in radians/s
wx = pi*alpha*cos(alpha*t*pi/180)*pi/180;
wy = pi*beta*cos(beta*t*pi/180)*pi/180;
wz = pi*gamma*cos(gamma*t*pi/180)*pi/180;


init_pos = [1;0;0];
pos = [1;0;0];

total_pos = [];
o = [ox; oy; oz]; %this is the evolution of rotations
p = [x; y; z];
for i= 1:size(o,2)
    o(:,i) = q2v(v2q(o(:,i)));
end

if isnan(o(1,1))
    o(1,1) = 0;
end
if isnan(o(2,1))
    o(2,1) = 0;
end
if isnan(o(3,1))
    o(3,1) = 0;
end

ox = o(1,:);
oy = o(2,:);
oz = o(3,:);

q_so3 = [];
%For each [ox; oy; oz] compute the corresponding 3D rotation matrix
%Apply this rotation matrix to the unit vector [1;0;0] to visualize the
%effect
for ii= 1:size(o,2)
    Rot = v2R(o(:,ii));
    q_so3 = [q_so3 v2q(o(:,ii))];
    pos = Rot*init_pos;
    total_pos = [total_pos pos];
end

u = [ax; ay; az; wx; wy; wz];
%% needed parameters

dt = 1/fe;
di = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
initial_condition = di;

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
u1 = [];

%FORMULATION IS PQV
%UNIT QUATERNION IS [1 0 0 0]

% We know how the angle varies --> i.e. we have the rate of turn
% components. The integrator deduces how the angle should vary from current
% state to next state. The angle information is stored in quaternion
for i=1:N*fe+1
    R0_1 = v2R(o(:,i));
    aR = inv(R0_1) * a0;
    u1(1:3,i) = inv(R0_1) * u(1:3,i) + aR;
    %u1(4:6,i) = inv(R0_1) * u(4:6,i);
    u1(4:6,i) = u(4:6,i);
    d = data2delta(b0, u1(:,i), n0, dt);
%% test imu_integrator

    di_out0 = imu_integrator(di, d, dt);
    di=di_out0;
    di_t = [di_t, di];
end

%% plot orientation over time
 qr = 4:7;
 orientation = [ox(1); oy(1); oz(1)]; %Will contain the positions of unit vector after rotation has been applied over time
 angle_reconstruct = []; % Will contain the rotation vector associated to quaternions
 Dq = [];
 
for j=1:size(di_t,2)
    q =  di_t(qr,j);
    pos = q2R(q)*init_pos;
    orientation = [orientation pos];
    angle_reconstruct = [angle_reconstruct q2v(q)];
    Dq = [Dq q];
end


% figure('Name','position plot - preintegrated','NumberTitle','off')
% subplot(3,2,1);
% plot(t, total_pos(1,:));
% hold on;
% plot(t, orientation(1,:),'r');
% xlabel('time');
% ylabel('pos x');
% legend('real orientation state', 'reconstructed orientation state');
% subplot(3,2,2);
% plot(t, orientation(1,:) - total_pos(1,:));
% xlabel('time');
% ylabel('pos x - ERROR');
% subplot(3,2,3);
% plot(t, total_pos(2,:));
% hold on;
% plot(t, orientation(2,:),'r');
% xlabel('time');
% ylabel('pos y');
% legend('real orientation state', 'reconstructed orientation state');
% subplot(3,2,4);
% plot(t, orientation(2,:) - total_pos(2,:));
% xlabel('time');
% ylabel('pos y - ERROR');
% subplot(3,2,5);
% plot(t, total_pos(3,:));
% hold on;
% plot(t, orientation(3,:),'r');
% xlabel('time');
% ylabel('pos z');
% legend('real orientation state', 'reconstructed orientation state');
% subplot(3,2,6);
% plot(t, orientation(3,:) - total_pos(3,:));
% xlabel('time');
% ylabel('pos z - ERROR');

%% error analysis
% error = total_pos - orientation;
% 
% 
% figure('Name','orientation error through time','NumberTitle','off');
% subplot(3,1,1);
% plot(t, error(1,:));
% xlabel('time');
% ylabel('error x');
% hold on;
% subplot(3,1,2);
% plot(t, error(2,:));
% xlabel('time');
% ylabel('error y');
% subplot(3,1,3);
% plot(t, error(3,:));
% xlabel('time');
% ylabel('error z');
% 
% figure('Name','orientation through time','NumberTitle','off');
% subplot(3,1,1);
% plot(t, angle_reconstruct(1,:));
% hold on;
% plot(t, ox(1,:),'r');
% xlabel('time');
% ylabel('angle x');
% legend('reconstructed orientation state', 'real orientation state');
% hold on;
% subplot(3,1,2);
% plot(t, angle_reconstruct(2,:));
% hold on;
% plot(t, oy(1,:),'r');
% xlabel('time');
% ylabel('angle y');
% legend('reconstructed orientation state', 'real orientation state');
% subplot(3,1,3);
% plot(t, angle_reconstruct(3,:));
% hold on;
% plot(t, oz(1,:),'r');
% xlabel('time');
% ylabel('angle z');
% legend('reconstructed orientation state', 'real orientation state');

% figure('Name','orientation through time- CPP','NumberTitle','off');
% subplot(3,1,1);
% plot(t, angles(6,:));
% xlabel('time');
% ylabel('angle x');
% legend('reconstructed orientation state');
% 
% subplot(3,1,2);
% plot(t, angles(7,:));
% xlabel('time');
% ylabel('angle y');
% legend('reconstructed orientation state');
% 
% subplot(3,1,3);
% plot(t, angles(8,:));
% xlabel('time');
% ylabel('angle z');
% legend('reconstructed orientation state');

%% check quaternions
figure('Name','compare quaternions','NumberTitle','off')
subplot(4,2,1);
plot(t, q_so3(2,:));
hold on;
plot(t, Dq(2,:),'r');
xlabel('time');
ylabel('qx');
legend('expected qx', 'estimated qx');

subplot(4,2,2);
plot(t, q_so3(2,:) - Dq(2,:));
xlabel('time');
ylabel('qx - ERROR');

subplot(4,2,3);
plot(t, q_so3(3,:));
hold on;
plot(t, Dq(3,:),'r');
xlabel('time');
ylabel('qy');
legend('expected qy', 'estimated qy');

subplot(4,2,4);
plot(t, q_so3(3,:) - Dq(3,:));
xlabel('time');
ylabel('qy - ERROR');

subplot(4,2,5);
plot(t, q_so3(4,:));
hold on;
plot(t, Dq(4,:),'r');
xlabel('time');
ylabel('qz');
legend('expected qz', 'estimated qz');

subplot(4,2,6);
plot(t, q_so3(4,:) - Dq(4,:));
xlabel('time');
ylabel('qz - ERROR');

subplot(4,2,7);
plot(t, q_so3(1,:));
hold on;
plot(t, Dq(1,:),'r');
xlabel('time');
ylabel('qw');
legend('expected qz', 'estimated qz');

subplot(4,2,8);
plot(t, q_so3(1,:) - Dq(1,:));
xlabel('time');
ylabel('qz - ERROR');

%% write in file

if(write_to_file)
    fileID = fopen('data_pure_rotation.txt','wt');
    data = [t',u1'];
    for ii = 1:size(data,1)
        fprintf(fileID,'%g\t',data(ii,:));
        fprintf(fileID,'\n');
    end
    fclose(fileID);
    disp('imu data written in file data_pure_rotation.txt');
    
    fileID_check = fopen('data_pure_rotation_check.txt','wt');
    data = [t',x',y',z', o'];
    for ii = 1:size(data,1)
        fprintf(fileID_check,'%g\t',data(ii,:));
        fprintf(fileID_check,'\n');
    end
    fclose(fileID_check);
    disp('data for checking written in file data_pure_rotation_check.txt');
end

if(write_to_file_const)
    fileID = fopen('data_pure_rotation.txt','wt');
    fprintf(fileID,'%g\t',initial_condition'); %initial condition is the first line of the file
    fprintf(fileID,'\n');
    data = [t',u1'];
    for ii = 1:size(data,1)
        fprintf(fileID,'%g\t',data(ii,:));
        fprintf(fileID,'\n');
    end
    fclose(fileID);
    
    fileID_check = fopen('odom_pure_rotation.txt','wt');
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