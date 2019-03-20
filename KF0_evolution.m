clear all;
close all;

% plot Keyframes state + 2*stdev from KeyFrame covariance
ref = load('./tests/1/KF0_evolution_initCorrectLong.dat');
ref_m = ref;
ref_m(:,5) = ref(:,8);
ref_m(:,8) = ref(:,5);

t_ref = ref_m(:,1);

data = load('./tests/1/KF0_evolutionGyro0_New01.dat'); %'./tests/1/KF0_evolutionAcc0_TEST.dat', './tests/1/KF0_evolutionAcc0_old1.dat'
data = load('./tests/1/tmp/KF0_evolution_old.dat');
data_m = data;
data_m(:,5) = data(:,8);
data_m(:,8) = data(:,5);

t = data_m(:,1);

data1 = load('./tests/1/KF0_evolutionAccGyro_New0001.dat');
data1 = load('./tests/1/tmp/KF0_evolution_new.dat');
data_m1 = data1;
data_m1(:,5) = data1(:,8);
data_m1(:,8) = data1(:,5);

t1 = data_m1(:,1);

%ranges
pr = 2:4;
qr = 5:8;
vr = 9:11;
abr = 12:14;
wbr = 15:17;

%% plots

%% position 3D
% figure('Name','3D position plot','NumberTitle','off');
% plot3(data_m(:,2),data_m(:,3),data_m(:,4), 'r');
% hold on;
% plot3(data_m1(:,2),data_m1(:,3),data_m1(:,4), 'b');
% plot3(data_m(1,2), data_m(1,3), data_m(1,4),'g*');
% xlabel('x posititon');
% ylabel('y posititon');
% zlabel('z posititon');
% legend('real Position State data0', 'real Position State data1', 'initial point');

%% position in  time

%position through time
% figure('Name','position through time','NumberTitle','off');
% subplot(3,1,1);
% plot(t, data_m(:,pr(1)), 'b');
% hold on;
% plot(t1, data_m1(:,pr(1)), 'r');
% xlabel('time');
% ylabel('position x');
% legend('integrated position data0', 'integrated position data1');
% 
% subplot(3,1,2);
% plot(t, data_m(:,pr(2)), 'b');
% hold on;
% plot(t1, data_m1(:,pr(2)), 'r');
% xlabel('time');
% ylabel('position y');
% legend('integrated position data0', 'integrated position data1');
% 
% subplot(3,1,3);
% plot(t, data_m(:,pr(3)), 'b');
% hold on;
% plot(t1, data_m1(:,pr(3)), 'r');
% xlabel('time');
% ylabel('position z');
% legend('integrated position data0', 'integrated position data1');

%% Quaternion

% figure('Name','quaternion through time','NumberTitle','off');
% subplot(4,1,1);
% plot(t, data_m(:,qr(1)), 'b');
% hold on;
% plot(t1, data_m1(:,qr(1)), 'r');
% xlabel('time');
% ylabel('quaternion w');
% legend('integrated quaternion data0', 'integrated quaternion data1');
% 
% subplot(4,1,2);
% plot(t, data_m(:,qr(2)), 'b');
% hold on;
% plot(t1, data_m1(:,qr(2)), 'r');
% xlabel('time');
% ylabel('quaternion x');
% legend('integrated quaternion data0', 'integrated quaternion data1');
% 
% subplot(4,1,3);
% plot(t, data_m(:,qr(3)), 'b');
% hold on;
% plot(t1, data_m1(:,qr(3)), 'r');
% xlabel('time');
% ylabel('quaternion y');
% legend('integrated quaternion data0', 'integrated quaternion data1');
% 
% subplot(4,1,4);
% plot(t, data_m(:,qr(4)), 'b');
% hold on;
% plot(t1, data_m1(:,qr(4)), 'r');
% xlabel('time');
% ylabel('quaternion z');
% legend('integrated quaternion data0', 'integrated quaternion data1');

%% Velocity

%velocity through time
% figure('Name','velocity through time','NumberTitle','off');
% subplot(3,1,1);
% plot(t, data_m(:,vr(1)), 'b');
% hold on;
% plot(t1, data_m1(:,vr(1)), 'r');
% plot(t_ref, ref_m(:,vr(1)), 'g');
% xlabel('time');
% ylabel('velocity x');
% legend('integrated velocity data0', 'integrated velocity data1', 'integrated velocity ref');
% 
% subplot(3,1,2);
% plot(t, data_m(:,vr(2)), 'b');
% hold on;
% plot(t1, data_m1(:,vr(2)), 'r');
% plot(t_ref, ref_m(:,vr(2)), 'g');
% xlabel('time');
% ylabel('velocity y');
% legend('integrated velocity data0', 'integrated velocity data1', 'integrated velocity ref');
% 
% subplot(3,1,3);
% plot(t, data_m(:,vr(3)), 'b');
% hold on;
% plot(t1, data_m1(:,vr(3)), 'r');
% plot(t_ref, ref_m(:,vr(3)), 'g');
% xlabel('time');
% ylabel('velocity z');
% legend('integrated velocity data0', 'integrated velocity data1', 'integrated velocity ref');


%% acclerometer bias

%acc bias through time
figure('Name','acc bias through time','NumberTitle','off');
plot(t, data_m(:,abr(1)), 'b');
hold on;
plot(t, data_m(:,abr(2)), 'g');
plot(t, data_m(:,abr(3)), 'r');
plot(t1, data_m1(:,abr(1)), 'b--');
plot(t1, data_m1(:,abr(2)), 'g--');
plot(t1, data_m1(:,abr(3)), 'r--');
plot(t_ref, ref_m(:,abr(1)), 'b:');
plot(t_ref, ref_m(:,abr(2)), 'g:');
plot(t_ref, ref_m(:,abr(3)), 'r:');
xlabel('time');
ylabel('accelerometer bias');
legend('estimated acc bias x', 'estimated acc bias y', 'estimated acc bias z' );
%% gyroscope bias

%gyro bias through time
figure('Name','gyro bias through time','NumberTitle','off');
plot(t, data_m(:,wbr(1)), 'b');
hold on;
plot(t, data_m(:,wbr(2)), 'g');
plot(t, data_m(:,wbr(3)), 'r');
plot(t1, data_m1(:,wbr(1)), 'b--');
plot(t1, data_m1(:,wbr(2)), 'g--');
plot(t1, data_m1(:,wbr(3)), 'r--');
plot(t_ref, ref_m(:,wbr(1)), 'b:');
plot(t_ref, ref_m(:,wbr(2)), 'g:');
plot(t_ref, ref_m(:,wbr(3)), 'r:');
xlabel('time');
ylabel('gyroscope bias');
legend('estimated gyro bias x', 'estimated gyro bias y', 'estimated gyro bias z' );