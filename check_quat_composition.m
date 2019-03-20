clear all;
close all;

% path_const_rot is path to file const_rot, this file contains data got
% from test_rotations in wolf. We have here successive rotations occuring
% at 1 KHz frequency with a constant rate of turn

% path_sin_rot0 is path to file sin_rot0, this file contains data got
% from test_rotations in wolf. We have here successive rotations occuring
% at 1 KHz frequency with a rate of turn defined by a cosinus function. We
% want this to match the reference orientation (following a sius function).
% All 3 axis vary here at the same frequency

% path_sin_rot is same as path_sin_rot0 but 1 of the axis is varying at a
% different frequency

% data structure : (N*fe)x7
% data(:,1) contains 'time axis'
% data(:,2) contains 'reference x angle (1st element of vector of rotation)'
% data(:,3) contains 'reference y angle (2st element of vector of rotation)'
% data(:,4) contains 'reference z angle (3st element of vector of rotation)'
% data(:,5) contains 'computed x angle (1st element of vector of rotation deduced from quaternion state)'
% data(:,6) contains 'computed y angle (2st element of vector of rotation deduced from quaternion state)'
% data(:,7) contains 'computed z angle (3st element of vector of rotation deduced from quaternion state)'


%% change these paths
path_sin_rot = './tests/sin_rot.dat';
path_sin_rot0 = './tests/sin_rot0.dat';
path_const_rot = './tests/const_rot.dat';


%% for sin_rot
data = load(path_sin_rot);

t = data(:,1);
ox_err = data(:,2) - data(:,5);
oy_err = data(:,3) - data(:,6);
oz_err = data(:,4) - data(:,7);

figure('Name','orientation through time - sin_rot','NumberTitle','off');
subplot(3,3,1);
plot(t, data(:,2));
xlabel('time');
ylabel('x (rad)');
hold on;
subplot(3,3,4);
plot(t, data(:,3));
xlabel('time');
ylabel('y (rad)');
subplot(3,3,7);
plot(t, data(:,4));
xlabel('time');
ylabel('z (rad)');

subplot(3,3,2);
plot(t, data(:,5));
xlabel('time');
ylabel('x (rad)');
legend('from quaternion composition');
subplot(3,3,5);
plot(t, data(:,6));
xlabel('time');
ylabel('y (rad)');
legend('from quaternion composition');
subplot(3,3,8);
plot(t, data(:,7));
xlabel('time');
ylabel('z (rad)');
legend('from quaternion composition');

subplot(3,3,3);
plot(t, ox_err(:,1));
xlabel('time');
ylabel('x error (rad)');
legend('desired - estimated');
subplot(3,3,6);
plot(t, oy_err(:,1));
xlabel('time');
ylabel('y error (rad)');
legend('desired - estimated');
subplot(3,3,9);
plot(t, oz_err(:,1));
xlabel('time');
ylabel('z error (rad)');
legend('desired - estimated');

%% for sin_rot0
data = load(path_sin_rot0);

t = data(:,1);
ox_err = data(:,2) - data(:,5);
oy_err = data(:,3) - data(:,6);
oz_err = data(:,4) - data(:,7);

figure('Name','orientation through time - sin_rot0','NumberTitle','off');
subplot(3,3,1);
plot(t, data(:,2));
xlabel('time');
ylabel('x (rad)');
hold on;
subplot(3,3,4);
plot(t, data(:,3));
xlabel('time');
ylabel('y (rad)');
subplot(3,3,7);
plot(t, data(:,4));
xlabel('time');
ylabel('z (rad)');

subplot(3,3,2);
plot(t, data(:,5));
xlabel('time');
ylabel('x (rad)');
legend('from quaternion composition');
subplot(3,3,5);
plot(t, data(:,6));
xlabel('time');
ylabel('y (rad)');
legend('from quaternion composition');
subplot(3,3,8);
plot(t, data(:,7));
xlabel('time');
ylabel('z (rad)');
legend('from quaternion composition');

subplot(3,3,3);
plot(t, ox_err(:,1));
xlabel('time');
ylabel('x error (rad)');
legend('desired - estimated');
subplot(3,3,6);
plot(t, oy_err(:,1));
xlabel('time');
ylabel('y error (rad)');
legend('desired - estimated');
subplot(3,3,9);
plot(t, oz_err(:,1));
xlabel('time');
ylabel('z error (rad)');
legend('desired - estimated');

%% for const_rot
data = load(path_const_rot);

t = data(:,1);
ox_err = data(:,2) - data(:,5);
oy_err = data(:,3) - data(:,6);
oz_err = data(:,4) - data(:,7);

figure('Name','orientation through time - const_rot','NumberTitle','off');
subplot(3,3,1);
plot(t, data(:,2));
xlabel('time');
ylabel('x (rad)');
hold on;
subplot(3,3,4);
plot(t, data(:,3));
xlabel('time');
ylabel('y (rad)');
subplot(3,3,7);
plot(t, data(:,4));
xlabel('time');
ylabel('z (rad)');

subplot(3,3,2);
plot(t, data(:,5));
xlabel('time');
ylabel('x (rad)');
legend('from quaternion composition');
subplot(3,3,5);
plot(t, data(:,6));
xlabel('time');
ylabel('y (rad)');
legend('from quaternion composition');
subplot(3,3,8);
plot(t, data(:,7));
xlabel('time');
ylabel('z (rad)');
legend('from quaternion composition');

subplot(3,3,3);
plot(t, ox_err(:,1));
xlabel('time');
ylabel('x error (rad)');
legend('desired - estimated');
subplot(3,3,6);
plot(t, oy_err(:,1));
xlabel('time');
ylabel('y error (rad)');
legend('desired - estimated');
subplot(3,3,9);
plot(t, oz_err(:,1));
xlabel('time');
ylabel('z error (rad)');
legend('desired - estimated');