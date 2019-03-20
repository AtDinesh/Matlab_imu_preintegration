close all;
clear a;;

fe = 1000;
N = 10*1;
t = (0:1/fe:N-1/fe);
st = sin(t);
ct = 0.5*sin(2*t);
zt = t;

figure;
plot3(st,ct,zt);
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');


[theta, phi, r] = cart2sph(st, ct, zt);

%% %DUMMY TEST1

close all;
clear a;;

fe = 1000;
N = 10*1;
t = (0:1/fe:N-1/fe);

ax(1,1:(N*fe)) = 0;
ay(1,1:(N*fe)) = 0;
az(1,1:(N*fe)) = 0;
wx(1,1:(N*fe)) = 0*deg_to_rad; 
wy(1,1:(N*fe)) = 0*deg_to_rad;
wz(1,1:(N*fe)) = 0*deg_to_rad;

ax(1,1) = 10;
ax(1,500) = -10;
ay(1,500) = 10;
ay(1,1000) = -10;
az(1,1000) = 10;
az(1,1500) = -10;
ay(1,1500) = -10;
ay(1,2000) = 10;
ax(1,2000) = -10;
ax(1,2500) = 10;
az(1,2500) = -10;
az(1,3000) = 10;

u = [ax; ay; az; wx; wy; wz];

%% needed parameters

dt = 0.001;
di = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
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

%% Plotting

%% Plolt Integrated position
figure('Name','position','NumberTitle','off');
subplot(3,1,1);
plot(t, di_t(1,:));
hold on;
subplot(3,1,2);
plot(t, di_t(2,:));
subplot(3,1,3);
plot(t, di_t(3,:));

%% 3D plot
figure('Name','3D position plot','NumberTitle','off');
plot3(di_t(1,:),di_t(2,:),di_t(3,:));
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');

%% all 3D plots in same figure :
figure('Name','compare trajectories','NumberTitle','off');
plot3(x(1,:),y(1,:),z(1,:), 'r');
hold on;
plot3(di_t(1,:),di_t(2,:),di_t(3,:), 'g');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');

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

%% %DUMMY TEST2

x = 
