clear all;
close all;
%% TEST FILE

%% generate data
fe = 100;
N = 30*50;
t = (0:1/fe:N-1/fe);
%t = (0:N)/fe;

f_ax = 1; %6;
f_ay = 1;
f_az = 1; %3;
f_wx = 1; %2;
f_wy = 10; %6;
f_wz = 1; %4;

deg_to_rad = 3.14159265359/180.0;
ax = 20*sin(2*pi*f_ax*t);
ay = 6*sin(2*pi*f_ay*t);
az = 6*sin(2*pi*f_az*t);
%wx(1,1:(N*fe)) = 1*deg_to_rad; 
%wy(1,1:(N*fe)) = 1*deg_to_rad;
wx = (180*sin(2*pi*f_wx*t))*deg_to_rad;
wy = (300*sin(2*pi*f_wy*t))*deg_to_rad;
wz = (180*sin(2*pi*f_wz*t))*deg_to_rad;
%wx = cos(2*pi*f_wx*t);
%wy = cos(2*pi*f_wy*t);
%wz = cos(2*pi*f_wz*t);

u = [ax; ay; az; wx; wy; wz];
%% needed parameters

dt = 0.001;
di = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
di0 = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
%u = [10; 5; 2; 110; 30; 50];
b0 = [0; 0; 0; 0; 0; 0]; %bias vector
n0 = [0; 0; 0; 0; 0; 0]; %noise vector

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
%% test delta preintegration
b = [0; 0; 0; 0; 0; 0]; %bias vector
n = [0; 0; 0; 0; 0; 0]; %noise vector

% for i=1:N*fe-1
%     [di_out, DI_OUT_di, DI_OUT_b, DI_OUT_u, DI_OUT_n] = integrateDelta (di0, b, u(:,1), n, dt);
%     %[di_out] = integrateDelta (di0, b, u(:,1), n, dt);
%     di0 = di_out;
%     di_t0 = [di_t0, di0];
% end
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

%% plot Integrated velocity
% figure('Name','velocity','NumberTitle','off');
% subplot(3,1,1);
% plot(t, di_t(8,:));
% hold on;
% subplot(3,1,2);
% plot(t, di_t(9,:));
% subplot(3,1,3);
% plot(t, di_t(10,:));

%% plot input data u
% figure('Name','input data','NumberTitle','off');
% subplot(3,1,1);
% plot(t, u(1,:));
% hold on;
% subplot(3,1,2);
% plot(t, u(2,:));
% subplot(3,1,3);
% plot(t, u(3,:));

%% plot delta velocity
% figure('Name','dv','NumberTitle','off');
% subplot(3,1,1);
% plot(t, u(1,:)*dt);
% hold on;
% subplot(3,1,2);
% plot(t, u(2,:)*dt);
% subplot(3,1,3);
% plot(t, u(3,:)*dt);

%% plot orientation over time
 qr = 4:7;
for j=1:size(di_t,2)
    q =  di_t(qr,j);
    orientation(:,j) = q2v(q);
end

% figure('Name','orientation','NumberTitle','off')
% subplot(3,1,1);
% plot(t, orientation(1,:)*(1.0/deg_to_rad));
% xlabel('x');
% hold on;
% subplot(3,1,2);
% plot(t, orientation(2,:)*(1.0/deg_to_rad));
% xlabel('y');
% subplot(3,1,3);
% plot(t, orientation(3,:)*(1.0/deg_to_rad));
% xlabel('z');

%% 3D plot
figure('Name','3D position plot','NumberTitle','off');
plot3(di_t(1,:),di_t(2,:),di_t(3,:));
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');