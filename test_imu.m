clear all;
close all;
%syms ax ay az wx wy wz abx aby abz wbx wby wbz dt real;
%syms dipx dipy dipz diqw diqx diqy diqz divx divy divz real;

%% LOAD DATA (IMU MEASUREMENTS)
data_acc = load('static_small_acc.dat');
data_gyro = load('static_small_gyro.dat');

if(size(data_acc,1) ~= size(data_gyro,1))
	disp('ERROR : data input file do not contain the same size of data ! Exiting...'); 
	return;
end

%u = [ax;ay;az;wx;wy;wz];
%di = [dipx;dipy;dipz;diqw;diqx;diqy;diqz;divx;divy;divz];
%b = [abx;aby;abz;wbx;wby;wbz];
di= [0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
b = [0; 0; 0.00; 0; 0; 0.00];

% u = rand(6,1);
% di = rand(10,1);
% b = rand(6,1);

%syms anx any anz wnx wny wnz real;
%n = [anx;any;anz;wnx;wny;wnz];
n = rand(6,1);
n = n/100;

%syms dt real;
dt = 0.001;

data_size = size(data_acc,1);
i=1;

fileID = fopen('matlab_debug_t2_nobias.txt', 'w');
%fprintf(fileID,'%%Timestamp\tD_px\tD_py\tD_pz\tD_qx\tD_qy\tD_qz\tD_qw\tD_vx\tD_vy\tD_vz\n');
fprintf(fileID,'%%Timestamp\tD_px\tD_py\tD_pz\tD_qx\tD_qy\tD_qz\tD_qw\tD_vx\tD_vy\tD_vz\td_px\td_py\td_pz\td_qx\td_qy\td_qz\td_qw\td_vx\td_vy\td_vz\n');

while(i <= data_size)
	u(1:3,1) = data_acc(i,2:4);
	u(4:6,1) = data_gyro(i,2:4);
	[di, DI_OUT_di, DI_OUT_b, DI_OUT_u, DI_OUT_n,d] = integrateDelta (di, b, u, n, dt);
    %fprintf(fileID,'%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n',data_acc(i,1),di(1,1),di(2,1),di(3,1),di(4,1),di(5,1),di(6,1),di(7,1),di(8,1),di(9,1),di(10,1));
    fprintf(fileID,'%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n',data_acc(i,1),di(1,1),di(2,1),di(3,1),di(4,1),di(5,1),di(6,1),di(7,1),di(8,1),di(9,1),di(10,1),d(1,1),d(2,1),d(3,1),d(4,1),d(5,1),d(6,1),d(7,1),d(8,1),d(9,1),d(10,1));
    i=i+1;
end

fclose(fileID);
disp('Delta Integration passed');

return;

%% PLOT DATA

debug = load('matlab_debug_t2_nobias.txt');

%% plot part 4 of quaternion wrt timestamp
figure('Name','orientation','NumberTitle','off');
subplot(2,1,1);
plot(debug(:,1), debug(:,4), 'b');
hold on;
plot(debug(:,1), debug(:,5), 'g');
plot(debug(:,1), debug(:,6), 'r');
plot(debug(:,1), debug(:,7), 'y');
ylabel('Dq'); xlabel('mti clock (10 KHz)');
legend('Dq_x', 'Dq_y', 'Dq_z','Dq_w');
title('Dq with timestamp');

subplot(2,1,2);
plot(data_gyro(:,1), data_gyro(:,2), 'b');
hold on;
plot(data_gyro(:,1), data_gyro(:,3), 'g');
plot(data_gyro(:,1), data_gyro(:,4), 'r');
ylabel('rad/sec'); xlabel('mti clock (10 KHz)');
legend('w_x', 'w_y', 'w_z');
title('gyroscope data');

%% plot velocities in debug file
figure('Name','Velocities','NumberTitle','off');
subplot(2,1,1);
plot(debug(:,1), debug(:,8), 'b');
hold on;
plot(debug(:,1), debug(:,9), 'g');
plot(debug(:,1), debug(:,10), 'r');
ylabel('m/sec'); xlabel('mti clock (10 KHz)');
legend('Dv_x', 'Dv_y', 'Dv_z')
title('integrated velocity');


subplot(2,1,2);
plot(data_gyro(:,1), data_gyro(:,2), 'b');
hold on;
plot(data_gyro(:,1), data_gyro(:,3), 'g');
plot(data_gyro(:,1), data_gyro(:,4), 'r');
ylabel('rad/sec'); xlabel('mti clock (10 KHz)');
legend('w_x', 'w_y', 'w_z');
title('gyroscope data');


%% plot deltas

figure('Name','deltas','NumberTitle','off');
subplot(3,1,1);
plot(debug(:,1), debug(:,12), 'b');
hold on;
plot(debug(:,1), debug(:,13), 'g');
plot(debug(:,1), debug(:,14), 'r');
ylabel('m/sec'); xlabel('mti clock (10 KHz)');
legend('Dp_x', 'Dp_y', 'Dp_z')
title('delta position');

subplot(3,1,2);
plot(debug(:,1), debug(:,15), 'b');
hold on;
plot(debug(:,1), debug(:,16), 'g');
plot(debug(:,1), debug(:,17), 'r');
plot(debug(:,1), debug(:,18), 'y');
ylabel('m/sec'); xlabel('mti clock (10 KHz)');
legend('Dq_w', 'Dq_x', 'Dq_y','Dq_z');
title('delta quaternion');

subplot(3,1,3);
plot(debug(:,1), debug(:,19), 'b');
hold on;
plot(debug(:,1), debug(:,20), 'g');
plot(debug(:,1), debug(:,21), 'r');
ylabel('m/sec'); xlabel('mti clock (10 KHz)');
legend('Dv_x', 'Dv_y', 'Dv_z')
title('delta velocity');

%% plot Integrated position

figure('Name','Velocities','NumberTitle','off');
subplot(2,1,1);
plot(debug(:,1), debug(:,2), 'b');
hold on;
plot(debug(:,1), debug(:,3), 'g');
plot(debug(:,1), debug(:,4), 'r');
ylabel('m'); xlabel('mti clock (10 KHz)');
legend('Dp_x', 'Dp_y', 'Dp_z')
title('integrated position');

subplot(2,1,2);
plot(data_gyro(:,1), data_acc(:,2), 'b');
hold on;
plot(data_gyro(:,1), data_acc(:,3), 'g');
plot(data_gyro(:,1), data_acc(:,4), 'r');
ylabel('m/sec^2'); xlabel('mti clock (10 KHz)');
legend('a_x', 'a_y', 'a_z');
title('accelerometer data');
