clearvars -except t3 state_vec x_final di_t b

data = load('./tests/framesCovariances13kf_new.dat');
data_m0 = data;
data_m0(:,5) = data(:,8);
data_m0(:,8) = data(:,5);

data_m0(:,21) = data(:,24);
data_m0(:,24) = data(:,21);

t = data_m0(:,1);

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

% plot res and ref

figure('Name','3D position plot','NumberTitle','off');
plot3(state_vec(1,:), state_vec(2,:), state_vec(3,:), 'g');
hold on;
plot3(data_m0(:,2),data_m0(:,3),data_m0(:,4), 'r');
plot3(data_m1(:,2),data_m1(:,3),data_m1(:,4), 'b');
plot3(data_m0(1,2), data_m0(1,3), data_m0(1,4),'g*');
plot3(x_final(1), x_final(2), x_final(3), 'r*');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('Reference Trajectory', 'Trajectory 7 Kf', 'Trajectory 121 Kf', 'initial position', 'final position');

%% position covariances
figure('Name','Position covariances (marginals)','NumberTitle','off');
subplot(3,1,1);
plot(t, data_m0(:,Cov_pr(1)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_pr(1)), 'm');
xlabel('time');
ylabel('2*stdev covX');
legend('2*stdev covX 7Kf', '2*stdev covX 121Kf');

subplot(3,1,2);
plot(t, data_m0(:,Cov_pr(2)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_pr(2)), 'm');
xlabel('time');
ylabel('2*stdev covY');
legend('2*stdev covY 7Kf', '2*stdev covY 121Kf');

subplot(3,1,3);
plot(t, data_m0(:,Cov_pr(3)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_pr(3)), 'm');
xlabel('time');
ylabel('2*stdev covZ');
legend('2*stdev covZ 7Kf', '2*stdev covZ 121Kf');

%% quaternion covariances
figure('Name','Quaternion covariances (marginals)','NumberTitle','off');
subplot(2,2,1);
plot(t, data_m0(:,Cov_qr(1)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_qr(1)), 'm');
xlabel('time');
ylabel('2*stdev covQw');
legend('2*stdev covQw 7Kf', '2*stdev covQw 121Kf');

subplot(2,2,2);
plot(t, data_m0(:,Cov_qr(2)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_qr(2)), 'm');
xlabel('time');
ylabel('2*stdev covQx');
legend('2*stdev covQx 7Kf', '2*stdev covQx 121Kf');

subplot(2,2,3);
plot(t, data_m0(:,Cov_qr(3)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_qr(3)), 'm');
xlabel('time');
ylabel('2*stdev covQy');
legend('2*stdev covQy 7Kf', '2*stdev covQy 121Kf');

subplot(2,2,4);
plot(t, data_m0(:,Cov_qr(4)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_qr(4)), 'm');
xlabel('time');
ylabel('2*stdev covQz');
legend('2*stdev covQz 7Kf', '2*stdev covQz 121Kf');

%% Velocity

%velocity through time
figure('Name','velocity covariances (marginals)','NumberTitle','off');
subplot(3,1,1);
plot(t, data_m0(:,Cov_vr(1)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_vr(1)), 'm');
xlabel('time');
ylabel('2*stdev covVx');
legend('2*stdev covVx 7Kf', '2*stdev covVx 121Kf');

subplot(3,1,2);
plot(t, data_m0(:,Cov_vr(2)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_vr(2)), 'm');
xlabel('time');
ylabel('2*stdev covVy');
legend('2*stdev covVy 7Kf', '2*stdev covVy 121Kf');

subplot(3,1,3);
plot(t, data_m0(:,Cov_vr(3)), 'b');
hold on;
plot(t_m1, data_m1(:,Cov_vr(3)), 'm');
xlabel('time');
ylabel('2*stdev covVz');
legend('2*stdev covVz 7Kf', '2*stdev covVz 121Kf');


