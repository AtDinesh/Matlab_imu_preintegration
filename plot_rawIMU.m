clear all;
close all;

data = load('./tests/pattern20s.dat');

time = data(:,1);
Ax = data(:,2);
Ay = data(:,3);
Az = data(:,4);
Wx = data(:,5);
Wy = data(:,6);
Wz = data(:,7);

figure('Name','plot IMU Acceleration data','NumberTitle','off');
plot(time, Ax, 'b');
hold on;
plot(time, Ay, 'g');
plot(time, Az, 'r');
xlabel('time (s)');
ylabel('Acc (m/s)');
legend('Ax', 'Ay', 'Az');

figure('Name','plot IMU Gyroscope data','NumberTitle','off');
plot(time, Wx, 'b');
hold on;
plot(time, Wy, 'g');
plot(time, Wz, 'r');
xlabel('time (s)');
ylabel('Gyro (rad/s)');
legend('Wx', 'Wy', 'Wz');