clear all;
close all;

% plot Keyframes state + 2*stdev from KeyFrame covariance

data = load('./tests/0/test_imu_constrained0_Long.dat');
data_m = data;
data_m(:,5) = data(:,8);
data_m(:,8) = data(:,5);

data_m(:,21) = data(:,24);
data_m(:,24) = data(:,21);

t = data_m(:,1);

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
plot3(data_m(:,2),data_m(:,3),data_m(:,4), 'r');
hold on;
plot3(data_m(1,2), data_m(1,3), data_m(1,4),'g*');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('real Position State', 'initial point');

%% position in  time

%position through time
figure('Name','position through time','NumberTitle','off');
subplot(3,2,1);
plot(t, data_m(:,pr(1)), 'b');
xlabel('time');
ylabel('position x');
legend('integrated position');

subplot(3,2,3);
plot(t, data_m(:,pr(2)), 'b');
xlabel('time');
ylabel('position y');
legend('integrated position');

subplot(3,2,5);
plot(t, data_m(:,pr(3)), 'b');
xlabel('time');
ylabel('position z');
legend('integrated position');

subplot(3,2,2);
plot(t, data_m(:,Cov_pr(1)), 'b');
xlabel('time');
ylabel('2*stdev covX');
legend('2*stdev covX');

subplot(3,2,4);
plot(t, data_m(:,Cov_pr(2)), 'b');
xlabel('time');
ylabel('2*stdev covY');
legend('2*stdev covY');

subplot(3,2,6);
plot(t, data_m(:,Cov_pr(3)), 'b');
xlabel('time');
ylabel('2*stdev covZ');
legend('2*stdev covZ');

%% Quaternion

figure('Name','quaternion through time','NumberTitle','off');
subplot(4,2,1);
plot(t, data_m(:,qr(1)), 'b');
xlabel('time');
ylabel('quaternion w');
legend('integrated quaternion');

subplot(4,2,3);
plot(t, data_m(:,qr(2)), 'b');
xlabel('time');
ylabel('quaternion x');
legend('integrated quaternion');

subplot(4,2,5);
plot(t, data_m(:,qr(3)), 'b');
xlabel('time');
ylabel('quaternion y');
legend('integrated quaternion');

subplot(4,2,7);
plot(t, data_m(:,qr(4)), 'b');
xlabel('time');
ylabel('quaternion z');
legend('integrated quaternion');

subplot(4,2,2);
plot(t, data_m(:,Cov_qr(1)), 'b');
xlabel('time');
ylabel('2*stdev covQw');
legend('2*stdev covQw');

subplot(4,2,4);
plot(t, data_m(:,Cov_qr(2)), 'b');
xlabel('time');
ylabel('2*stdev covQx');
legend('2*stdev covQx');

subplot(4,2,6);
plot(t, data_m(:,Cov_qr(3)), 'b');
xlabel('time');
ylabel('2*stdev covQy');
legend('2*stdev covQy');

subplot(4,2,8);
plot(t, data_m(:,Cov_qr(4)), 'b');
xlabel('time');
ylabel('2*stdev covQz');
legend('2*stdev covQz');

%% Velocity

%velocity through time
figure('Name','velocity through time','NumberTitle','off');
subplot(3,2,1);
plot(t, data_m(:,vr(1)), 'b');
xlabel('time');
ylabel('velocity x');
legend('integrated velocity');

subplot(3,2,3);
plot(t, data_m(:,vr(2)), 'b');
xlabel('time');
ylabel('velocity y');
legend('integrated velocity');

subplot(3,2,5);
plot(t, data_m(:,vr(3)), 'b');
xlabel('time');
ylabel('velocity z');
legend('integrated velocity');

subplot(3,2,2);
plot(t, data_m(:,Cov_vr(1)), 'b');
xlabel('time');
ylabel('2*stdev covVx');
legend('2*stdev covVx');

subplot(3,2,4);
plot(t, data_m(:,Cov_vr(2)), 'b');
xlabel('time');
ylabel('2*stdev covVy');
legend('2*stdev covVy');

subplot(3,2,6);
plot(t, data_m(:,Cov_vr(3)), 'b');
xlabel('time');
ylabel('2*stdev covVz');
legend('2*stdev covVz');

%% acclerometer bias

%acc bias through time
figure('Name','acc bias through time','NumberTitle','off');
subplot(2,2,1);
plot(t, data_m(:,abr(1)), 'b');
hold on;
plot(t, data_m(:,abr(2)), 'g');
plot(t, data_m(:,abr(3)), 'r');
xlabel('time');
ylabel('accelerometer bias');
legend('estimated accelerometer bias x', 'estimated accelerometer bias y', 'estimated accelerometer bias z' );

subplot(2,2,2);
plot(t, data_m(:,Cov_abr(1)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covABx');

subplot(2,2,3);
plot(t, data_m(:,Cov_abr(2)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covABy');

subplot(2,2,4);
plot(t, data_m(:,Cov_abr(3)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covABz');

%% gyroscope bias

%gyro bias through time
figure('Name','gyro bias through time','NumberTitle','off');
subplot(2,2,1);
plot(t, data_m(:,wbr(1)), 'b');
hold on;
plot(t, data_m(:,wbr(2)), 'g');
plot(t, data_m(:,wbr(3)), 'r');
xlabel('time');
ylabel('gyroscope bias');
legend('estimated gyroscope bias x', 'estimated gyroscope bias y', 'estimated gyroscope bias z' );

subplot(2,2,2);
plot(t, data_m(:,Cov_wbr(1)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covWBx');

subplot(2,2,3);
plot(t, data_m(:,Cov_wbr(2)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covWBy');

subplot(2,2,4);
plot(t, data_m(:,Cov_wbr(3)), 'b');
xlabel('time');
ylabel('2*stdev');
legend('2*stdev covWBz');