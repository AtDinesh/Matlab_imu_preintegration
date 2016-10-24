clear all;
close all;

% fe = 1000;
% N = 10*1;
% t = (0:1/fe:N-1/fe);
% st = sin(t);
% ct = 0.5*cos(2*t);
% zt = cos(3*t);
% 
% figure
% plot3(st,ct,zt)


fe = 1000;
N = 10*1;
t = (0:1/fe:N-1/fe);

%% rotate around random direction
% rot = rand(4,N*fe);
% R = [1,0,0;0,1,0;0,0,1];
% dat_R = [0;0;0];
% 
% for i=1:N*fe-1
%     R = R * rotationmat3D(rot(1,i),[rot(2,i),rot(3,i),rot(4,i)]);
%     dat_R = [dat_R q2v(R2q(R))];
% end
% 
% f_wx = 1;
% f_wy = 0.5;
% f_wz = 1.5;
% 
% ox = cos(f_wx*t);
% oy = cos(f_wy*t);
% oz = cos(f_wz*t);
% 
% 
% figure('Name','orientation','NumberTitle','off')
% subplot(3,1,1);
% plot(t,ox(1,:));
% xlabel('x');
% hold on;
% subplot(3,1,2);
% plot(t,oy(1,:));
% xlabel('y');
% subplot(3,1,3);
% plot(t,oz(1,:));
% xlabel('z');

%% ex

r = 1;
alpha = 1*2*pi;
beta = 0.5*2*pi;

fe = 1000;
N = 10*1;
t = (0:1/fe:N-1/fe);

%PSI = alpha*t (alpha = a*2*pi)
%THETA = beta*t (alpha = b*2*pi)
X = r*cos(alpha*t).*cos(beta*t);
Y = r*cos(alpha*t).*sin(beta*t);
Z = r*sin(alpha*t);
figure('Name','cos and sin','NumberTitle','off')
subplot(3,2,1);
plot(t,cos(alpha*t));
xlabel('cos(alpha*t)');
hold on;
subplot(3,2,2);
plot(t,sin(alpha*t));
xlabel('sin(alpha*t) : Oz');
subplot(3,2,3);
plot(t,cos(beta*t));
xlabel('cos(beta*t) : Ox');
subplot(3,2,4);
plot(t,sin(beta*t));
xlabel('sin(beta*t) : Oy');
subplot(3,2,5);
plot(t,X(1,:));
xlabel('X');
subplot(3,2,6);
plot(t,Y(1,:));
xlabel('Y');

ox = cos(beta*t);
oy = sin(beta*t);
oz = sin(alpha*t);

v = [ox(1,:); oy(1,:); oz(1,:)]; %this is the evolution of rotations
rot_mat = [1,0,0;0,1,0;0,0,1];
for ii = 1:size(v,2)
    rot = v2R(v(:,ii));
    rot_mat = [rot_mat, rot];
end

%% derivatives :
%these are velocity of the point (position)
x_d = -alpha*sin(alpha*t).*cos(beta*t) - beta*cos(alpha*t).*sin(beta*t);
y_d = -alpha*sin(alpha*t).*sin(beta*t) + beta*cos(alpha*t).*cos(beta*t);
z_d = alpha*cos(alpha*t);

wx = -beta * sin(beta*t);
wy = beta * cos(beta*t);
wz = alpha * cos(alpha*t);
%acceleration (position)
ax(1,1:(N*fe)) = 0;
ay(1,1:(N*fe)) = 0;
az(1,1:(N*fe)) = 0;

u = [ax; ay; az; wx; wy; wz];

%% needed parameters

dt = 0.001;
di = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
di(4:7,1) = v2q([1;0;0]);
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

%% plot orientation over time
 qr = 4:7;
 orientation = [];
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
