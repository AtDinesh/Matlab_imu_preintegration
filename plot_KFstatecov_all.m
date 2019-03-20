clear all;
close all;

% plot Keyframes state + 2*stdev from KeyFrame covariance

%data = load('./tests/0/debug_results_imu_constrained0_600KF_OdomKhz.dat');
data = load('./tests/framesCovariances13kf_new.dat');
data_m0 = data;
data_m0(:,5) = data(:,8);
data_m0(:,8) = data(:,5);

data_m0(:,21) = data(:,24);
data_m0(:,24) = data(:,21);

t = data_m0(:,1);

%data = load('./tests/0/debug_results_imu_constrained0_6KF_OdomKHz.dat');
data = load('./tests/framesCovariances121kf_new.dat');
data_m1 = data;
data_m1(:,5) = data(:,8);
data_m1(:,8) = data(:,5);

data_m1(:,21) = data(:,24);
data_m1(:,24) = data(:,21);

t_m1 = data_m1(:,1);

%ranges
pr = 2:4;
qr = 5:8;
vr = 9:11;
abr = 12:14;
wbr = 15:17;

Cov_pr = 18:20;
Cov_qr = 21:24;
Cov_vr = 25:27;
Cov_abr = 28:30;
Cov_wbr = 31:33;

%% plots

%% position 3D
figure('Name','3D position plot','NumberTitle','off');
plot3(data_m0(:,2),data_m0(:,3),data_m0(:,4), 'r');
hold on;
plot3(data_m1(:,2),data_m1(:,3),data_m1(:,4), 'm');
plot3(data_m0(1,2), data_m0(1,3), data_m0(1,4),'g*');
plot3(data_m0(size(data_m0,1),2), data_m0(size(data_m0,1),3), data_m0(size(data_m0,1),4),'m*');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('KF Position State 200Hz', 'KF Position State 2Hz', 'initial position', 'final position');

%% position in  time

%position through time
figure('Name','position through time','NumberTitle','off');
subplot(3,2,1);
plot(t, data_m0(:,pr(1)), 'b');
hold on;
plot(t_m1, data_m1(:,pr(1)), 'm');
xlabel('time');
ylabel('position x');
legend('KF position 200 Hz', 'KF position 2 Hz');

subplot(3,2,3);
plot(t, data_m0(:,pr(2)), 'b');
hold on;
plot(t_m1, data_m1(:,pr(2)), 'm');
xlabel('time');
ylabel('position y');
legend('KF position 200 Hz', 'KF position 2 Hz');

subplot(3,2,5);
plot(t, data_m0(:,pr(3)), 'b');
hold on;
plot(t_m1, data_m1(:,pr(3)), 'm');
xlabel('time');
ylabel('position z');
legend('KF position 200 Hz', 'KF position 2 Hz');

subplot(3,2,2);
plot(t, data_m0(:,Cov_pr(1)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_pr(1)), 'm');
xlabel('time');
ylabel('2*stdev covX');
legend('2*stdev covX 200Hz', '2*stdev covX 2 Hz');

subplot(3,2,4);
plot(t, data_m0(:,Cov_pr(2)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_pr(2)), 'm');
xlabel('time');
ylabel('2*stdev covY');
legend('2*stdev covY 200Hz', '2*stdev covY 2 Hz');

subplot(3,2,6);
plot(t, data_m0(:,Cov_pr(3)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_pr(3)), 'm');
xlabel('time');
ylabel('2*stdev covZ');
legend('2*stdev covZ 200 Hz', '2*stdev covZ 2 Hz');

%% Quaternion

figure('Name','quaternion through time','NumberTitle','off');
subplot(4,2,1);
plot(t, data_m0(:,qr(1)), 'b');
hold on;
plot(t_m1, data_m1(:,qr(1)), 'm');
xlabel('time');
ylabel('quaternion w');
legend('KF quaternion 200Hz', 'KF quaternion 2Hz');

subplot(4,2,3);
plot(t, data_m0(:,qr(2)), 'b');
hold on;
plot(t_m1, data_m1(:,qr(2)), 'm');
xlabel('time');
ylabel('quaternion x');
legend('KF quaternion 200Hz', 'KF quaternion 2Hz');

subplot(4,2,5);
plot(t, data_m0(:,qr(3)), 'b');
hold on;
plot(t_m1, data_m1(:,qr(3)), 'm');
xlabel('time');
ylabel('quaternion y');
legend('KF quaternion 200Hz', 'KF quaternion 2Hz');

subplot(4,2,7);
plot(t, data_m0(:,qr(4)), 'b');
hold on;
plot(t_m1, data_m1(:,qr(4)), 'm');
xlabel('time');
ylabel('quaternion z');
legend('KF quaternion 200Hz', 'KF quaternion 2Hz');

subplot(4,2,2);
plot(t, data_m0(:,Cov_qr(1)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_qr(1)), 'm');
xlabel('time');
ylabel('2*stdev covQw');
legend('2*stdev covQw  200 Hz', '2*stdev covQw  2 Hz');

subplot(4,2,4);
plot(t, data_m0(:,Cov_qr(2)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_qr(2)), 'm');
xlabel('time');
ylabel('2*stdev covQx');
legend('2*stdev covQx  200 Hz', '2*stdev covQx  2 Hz');

subplot(4,2,6);
plot(t, data_m0(:,Cov_qr(3)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_qr(3)), 'm');
xlabel('time');
ylabel('2*stdev covQy');
legend('2*stdev covQy  200 Hz', '2*stdev covQy  2 Hz');

subplot(4,2,8);
plot(t, data_m0(:,Cov_qr(4)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_qr(4)), 'm');
xlabel('time');
ylabel('2*stdev covQz');
legend('2*stdev covQz  200 Hz', '2*stdev covQz  2 Hz');

%% Velocity

%velocity through time
figure('Name','velocity through time','NumberTitle','off');
subplot(3,2,1);
plot(t, data_m0(:,vr(1)), 'b');
hold on;
plot(t_m1, data_m1(:,vr(1)), 'm');
xlabel('time');
ylabel('velocity x');
legend('KF velocity 200 Hz', 'KF velocity 2 Hz');

subplot(3,2,3);
plot(t, data_m0(:,vr(2)), 'b');
hold on;
plot(t_m1, data_m1(:,vr(2)), 'm');
xlabel('time');
ylabel('velocity y');
legend('KF velocity 200 Hz', 'KF velocity 2 Hz');

subplot(3,2,5);
plot(t, data_m0(:,vr(3)), 'b');
hold on;
plot(t_m1, data_m1(:,vr(3)), 'm');
xlabel('time');
ylabel('velocity z');
legend('KF velocity 200 Hz', 'KF velocity 2 Hz');

subplot(3,2,2);
plot(t, data_m0(:,Cov_vr(1)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_vr(1)), 'm');
xlabel('time');
ylabel('2*stdev covVx');
legend('2*stdev covVx 200 Hz', '2*stdev covVx 2 Hz');

subplot(3,2,4);
plot(t, data_m0(:,Cov_vr(2)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_vr(2)), 'm');
xlabel('time');
ylabel('2*stdev covVy');
legend('2*stdev covVy 200 Hz', '2*stdev covVy 2 Hz');

subplot(3,2,6);
plot(t, data_m0(:,Cov_vr(3)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_vr(3)), 'm');
xlabel('time');
ylabel('2*stdev covVz');
legend('2*stdev covVz 200 Hz', '2*stdev covVz 2 Hz');

%% acclerometer bias

%abthrough time
figure('Name','acc bias through time','NumberTitle','off');
subplot(4,2,1);
plot(t, data_m0(:,abr(1)), 'b');
hold on;
plot(t, data_m0(:,abr(2)), 'g');
plot(t, data_m0(:,abr(3)), 'r');
xlabel('time');
ylabel('accelerometer bias');
legend('estim. abx 200Hz', 'estim. aby 200Hz', 'estim. abz 200Hz' );

subplot(4,2,3);
plot(t, data_m0(:,Cov_abr(1)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covABx');

subplot(4,2,5);
plot(t, data_m0(:,Cov_abr(2)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covABy');

subplot(4,2,7);
plot(t, data_m0(:,Cov_abr(3)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covABz');

%________________________________________2Hz KF
subplot(4,2,2);
plot(t_m1, data_m1(:,abr(1)), 'b');
hold on;
plot(t_m1, data_m1(:,abr(2)), 'g');
plot(t_m1, data_m1(:,abr(3)), 'r');
xlabel('time');
ylabel('accelerometer bias');
legend('estim. abx 2Hz', 'estim. aby 2Hz', 'estim. abz 2Hz' );

subplot(4,2,4);
plot(t_m1, data_m1(:,Cov_abr(1)), 'm');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covABx 2 Hz');

subplot(4,2,6);
plot(t_m1, data_m1(:,Cov_abr(2)), 'm');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covABy 2 Hz');

subplot(4,2,8);
plot(t_m1, data_m1(:,Cov_abr(3)), 'm');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covABz 2 Hz');

%% gyroscope bias

%wbthrough time
figure('Name','gyro bias through time','NumberTitle','off');
subplot(4,2,1);
plot(t, data_m0(:,wbr(1)), 'b');
hold on;
plot(t, data_m0(:,wbr(2)), 'g');
plot(t, data_m0(:,wbr(3)), 'r');
xlabel('time');
ylabel('gyroscope bias');
legend('estim. wbx 200Hz', 'estim. wby 200Hz', 'estim. wbz 200Hz' );

subplot(4,2,3);
plot(t, data_m0(:,Cov_wbr(1)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covWBx 200 Hz');

subplot(4,2,5);
plot(t, data_m0(:,Cov_wbr(2)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covWBy 200 Hz');

subplot(4,2,7);
plot(t, data_m0(:,Cov_wbr(3)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covWBz 200 Hz');

%_______________________ 2Hz

subplot(4,2,2);
plot(t_m1, data_m1(:,wbr(1)), 'b');
hold on;
plot(t_m1, data_m1(:,wbr(2)), 'g');
plot(t_m1, data_m1(:,wbr(3)), 'r');
xlabel('time');
ylabel('gyroscope bias');
legend('estim. wbx 2Hz', 'estim. wby 2Hz', 'estim. wbz 2Hz' );

subplot(4,2,4);
plot(t_m1, data_m1(:,Cov_wbr(1)), 'm');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covWBx 2 Hz');

subplot(4,2,6);
plot(t_m1, data_m1(:,Cov_wbr(2)), 'm');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covWBy 2 Hz');

subplot(4,2,8);
plot(t_m1, data_m1(:,Cov_wbr(3)), 'm');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covWBz 2 Hz');