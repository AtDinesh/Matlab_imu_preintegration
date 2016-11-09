%% WOLF::PROCESSOR_IMU VS MATLAB->IMU_INTEGRATOR
%in this file we use data from C++ processor_imu and feed matlab perfect imu 
%integrator with simulation data to reconstruct the pose of the imu to
%compare both integrator.

%%
clear all;
close all;

%               SET INITIAL CONDITION FOR MATLAB PREINTEGRATION HERE
di = [0; 0; 0; 1; 0; 0; 0; 1; 2; 5];

%load files
test = load('./data_files/rotation_plus_translation/0/debug_results1.dat'); % data from C++ integrator
check_data = load('./data_files/rotation_plus_translation/0/data_final_test_check.txt'); % in PQV formulation contains successive state from matlab
imu_data = load('./data_files/rotation_plus_translation/0/data_final_test.txt');

%ranges
delta_range = 2:11;
Delta_range = 12:21; %File in PVQ
State_range = 22:31;
cpp_qr = 7:10;+
matlab_qr = 4:7;
cpp_pr = 1:3;
cpp_vr = 4:6;
mat_pr = 2:4;
mat_vr = 8:10;
or = 5:7;

%get data
test_delta = test(:,delta_range);
%test_delta(:,7) = test(:,11);
%test_delta(:,10) = test(:,8);
test_Delta = test(:,Delta_range);
%test_Delta(:,7) = test(:,21);
%test_Delta(:,10) = test(:,18);
test_State = test(:,State_range);
%test_State(:,7) = test(:,31);
%test_State(:,10) = test(:,28);

ref_pos = check_data(:,mat_pr);
ref_or = check_data(:,or);

cpp_or_delta = [];
cpp_or_Delta = [];
cpp_or_State = [];
cpp_pos_Delta = [];
cpp_pos_State = [];
cpp_vel_Delta = [];

% in Eigen, quaternons as contructed using method Quaternion(w,x,y,z) but
% stored internally as [x,y,z,w]. In Matlab, the unit quaternion (from the
% toolbox that we use) is [1 0 0 0]. Thus to make things match we have to
% be careful to this convention thing but reordering data we get from CPP
% program.
for(i=1:size(test,1))
    dq_ = test_delta(i,cpp_qr);
    dq = [dq_(1,4) dq_(1,1) dq_(1,2) dq_(1,3)];
    Dq_ = test_Delta(i,cpp_qr);
    Dq = [Dq_(1,4) Dq_(1,1) Dq_(1,2) Dq_(1,3)];
    Xq_ = test_State(i,cpp_qr);
    Xq = [Xq_(1,4) Xq_(1,1) Xq_(1,2) Xq_(1,3)];
    cpp_or_delta = [cpp_or_delta q2v(dq')];
    cpp_or_Delta = [cpp_or_Delta q2v(Dq')];
    cpp_or_State = [cpp_or_State q2v(Xq')];
end

cpp_pos_Delta = test_Delta(:,cpp_pr);
cpp_pos_State = test_State(:,cpp_pr);
cpp_vel_Delta = test_Delta(:,cpp_vr);
t = check_data(:,1);

% adjust data size
% cpp_pos_Delta = cpp_pos_Delta(1:size(cpp_pos_Delta,1)-1,:);
% cpp_pos_State = cpp_pos_State(1:size(cpp_pos_State,1)-1,:);
% cpp_vel_Delta = cpp_vel_Delta(1:size(cpp_vel_Delta,1)-1,:);
% cpp_or_delta = cpp_or_delta(:,1:size(cpp_or_delta,2)-1);
% cpp_or_Delta = cpp_or_Delta(:,1:size(cpp_or_Delta,2)-1);
% cpp_or_State = cpp_or_State(:,1:size(cpp_or_State,2)-1);
% test = test(1:size(test,1)-1,:);
% test_delta = test_delta(1:size(test_delta,1)-1,:);
% test_Delta = test_Delta(1:size(test_Delta,1)-1,:);
% test_State = test_State(1:size(test_State,1)-1,:);

cpp_pos_Delta = cpp_pos_Delta(1:size(cpp_pos_Delta,1),:);
cpp_pos_State = cpp_pos_State(1:size(cpp_pos_State,1),:);
cpp_vel_Delta = cpp_vel_Delta(1:size(cpp_vel_Delta,1),:);
cpp_or_delta = cpp_or_delta(:,1:size(cpp_or_delta,2));
cpp_or_Delta = cpp_or_Delta(:,1:size(cpp_or_Delta,2));
cpp_or_State = cpp_or_State(:,1:size(cpp_or_State,2));
test = test(1:size(test,1),:);
test_delta = test_delta(1:size(test_delta,1),:);
test_Delta = test_Delta(1:size(test_Delta,1),:);
test_State = test_State(17:size(test_State,1),:);

%% feed matlab simulator
fe = 1000;
dt = 1/fe;
b0 = [0; 0; 0; 0; 0; 0]; %bias vector
n0 = [0; 0; 0; 0; 0; 0]; %noise vector
di_t = [];
total_pos = [];

%reconstruct trajectory
for(i=1:size(imu_data,1))
    u = imu_data(i,2:7);
    d = data2delta(b0, u', n0, dt);
    di= imu_integrator(di, d, dt);
    di_t = [di_t, di];
end

di_t = di_t';

%% PLOTS

%% position
%position 3D
figure('Name','3D position plot','NumberTitle','off');
plot3(check_data(:,2),check_data(:,3),check_data(:,4), 'g');
hold on;
plot3(cpp_pos_Delta(:,1), cpp_pos_Delta(:,2), cpp_pos_Delta(:,3), 'r');
plot3(di_t(:,1), di_t(:,2), di_t(:,3), 'm');
plot3(0,0,0,'g*');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('real Position State', 'integrated Delta Position', 'matlab integration');

figure('Name','3D position plot','NumberTitle','off');
plot3(check_data(:,2),check_data(:,3),check_data(:,4), 'g');
hold on;
plot3(di_t(:,1), di_t(:,2), di_t(:,3), 'm');
plot3(0,0,0,'g*');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('real Position State', 'integrated Delta Position', 'matlab integration');

%position error
pos_error = [check_data(:,2)-cpp_pos_Delta(:,1) check_data(:,3)-cpp_pos_Delta(:,2) check_data(:,4)-cpp_pos_Delta(:,3)];
%position through time
figure('Name','position through time','NumberTitle','off');
subplot(3,2,1);
plot(t, check_data(:,2), 'g');
hold on;
plot(t, cpp_pos_Delta(:,1),'r');
xlabel('time');
ylabel('position x');
legend('Reference position', 'integrated Delta position');

subplot(3,2,3);
plot(t, check_data(:,3), 'g');
hold on;
plot(t, cpp_pos_Delta(:,2),'r');
xlabel('time');
ylabel('position y');
legend('Reference position', 'integrated Delta position');

subplot(3,2,5);
plot(t, check_data(:,4), 'g');
hold on;
plot(t, cpp_pos_Delta(:,3),'r');
xlabel('time');
ylabel('position z');
legend('Reference position', 'integrated Delta position');

subplot(3,2,2);
plot(t, pos_error(:,1));
xlabel('time');
ylabel('error in position x');
legend('error in position x');

subplot(3,2,4);
plot(t, pos_error(:,2));
xlabel('time');
ylabel('error in position y');
legend('error in position y');

subplot(3,2,6);
plot(t, pos_error(:,3));
xlabel('time');
ylabel('error in position z');
legend('error in position z');

%% velocity
velocity_error = [di_t(:,8)-cpp_vel_Delta(:,1) di_t(:,9)-cpp_vel_Delta(:,2) di_t(:,10)-cpp_vel_Delta(:,3)];
%Velocity
figure('Name','Velocity through time','NumberTitle','off');
subplot(3,2,1);
plot(t, cpp_vel_Delta(:,1), 'g');
hold on;
plot(t, di_t(:,8),'m');
xlabel('time');
ylabel('velocity X');
legend('cpp velocity X', 'matlab velocity X');

subplot(3,2,3);
plot(t, cpp_vel_Delta(:,2), 'g');
hold on;
plot(t, di_t(:,9),'m');
xlabel('time');
ylabel('velocity Y');
legend('cpp velocity Y', 'matlab velocity Y');

subplot(3,2,5);
plot(t, cpp_vel_Delta(:,3), 'g');
hold on;
plot(t, di_t(:,10),'m');
xlabel('time');
ylabel('velocity Z');
legend('cpp velocity Y', 'matlab velocity Y');

subplot(3,2,2);
plot(t, velocity_error(:,1), 'r');
hold on;
plot(t, ones(size(t))*mean(velocity_error(:,1), 'double'));
xlabel('time');
ylabel('velocity error X');
legend('Velocity Error','velocity error mean');

subplot(3,2,4);
plot(t, velocity_error(:,2), 'r');
hold on;
plot(t, ones(size(t))*mean(velocity_error(:,2), 'double'));
xlabel('time');
ylabel('velocity error Y');
legend('Velocity Error', 'velocity error mean');

subplot(3,2,6);
plot(t, velocity_error(:,3), 'r');
hold on;
plot(t, ones(size(t))*mean(velocity_error(:,3), 'double'));
xlabel('time');
ylabel('velocity error Z');
legend('Velocity Error', 'velocity error mean');



%% orientation
%orientation
figure('Name','orientation through time','NumberTitle','off');
subplot(3,2,1);
plot(t, check_data(:,5), 'g');
hold on;
plot(t, cpp_or_Delta(1,:),'r');
xlabel('time');
ylabel('angle x');
legend('Real orientation state', 'integrated Delta orientation state');

hold on;
subplot(3,2,3);
plot(t, check_data(:,6), 'g');
hold on;
plot(t, cpp_or_Delta(2,:),'r');
xlabel('time');
ylabel('angle y');
legend('reconstructed orientation state', 'integrated Delta orientation state');

subplot(3,2,5);
plot(t, check_data(:,7), 'g');
hold on;
plot(t, cpp_or_Delta(3,:),'r');
xlabel('time');
ylabel('angle z');
legend('reconstructed orientation state', 'integrated Delta orientation state');

%error in orientation
orientation_error = [check_data(:,5)-cpp_or_Delta(1,:)' check_data(:,6)-cpp_or_Delta(2,:)' check_data(:,7)-cpp_or_Delta(3,:)'];

subplot(3,2,2);
plot(t, orientation_error(:,1));
xlabel('time');
ylabel('angle x Error');
legend('Error in Angle X');

subplot(3,2,4);
plot(t, orientation_error(:,2));
xlabel('time');
ylabel('angle y Error');
legend('Error in Angle Y');

subplot(3,2,6);
plot(t, orientation_error(:,3));
xlabel('time');
ylabel('angle z Error');
legend('Error in Angle Z');

%evolution of instantaneous delta orientation
% figure('Name','Velocity through time','NumberTitle','off');
% subplot(3,1,1);
% plot(t, cpp_or_delta(1,:));
% hold on;
% xlabel('time');
% ylabel('delta orientation x');
% legend('Instantaneous delta orientation x');
% 
% subplot(3,1,2);
% plot(t, cpp_or_delta(2,:));
% hold on;
% xlabel('time');
% ylabel('delta orientation y');
% legend('Instantaneous delta orientation y');
% 
% subplot(3,1,3);
% plot(t, cpp_or_delta(3,:));
% hold on;
% xlabel('time');
% ylabel('delta orientation z');
% legend('Instantaneous delta orientation z');

%% compare quaternions
quaternion_errors = [di_t(:,4)-test_Delta(:,10) di_t(:,5)-test_Delta(:,7) di_t(:,6)-test_Delta(:,8) di_t(:,7)-test_Delta(:,9)];
figure('Name','Compare quaternions','NumberTitle','off');
subplot(4,2,1);
plot(t,di_t(:,4),'r');
hold on;
plot(t,test_Delta(:,10),'g');
legend('matlab integration', 'cpp integration')
xlabel('time');
ylabel('scalar part');

subplot(4,2,2);
plot(t,di_t(:,5),'r');
hold on;
plot(t,test_Delta(:,7),'g');
legend('matlab integration', 'cpp integration')
xlabel('time');
ylabel('qx');

subplot(4,2,3);
plot(t,di_t(:,6),'r');
hold on;
plot(t,test_Delta(:,8),'g');
legend('matlab integration', 'cpp integration')
xlabel('time');
ylabel('qy');

subplot(4,2,4);
plot(t,di_t(:,7),'r');
hold on;
plot(t,test_Delta(:,9),'g');
legend('matlab integration', 'cpp integration')
xlabel('time');
ylabel('qy');

subplot(4,2,5);
plot(t,quaternion_errors(:,1),'r');
legend('quaternion error')
xlabel('time');
ylabel('scalar');

subplot(4,2,6);
plot(t,quaternion_errors(:,2),'r');
legend('quaternion error')
xlabel('time');
ylabel('qx');

subplot(4,2,7);
plot(t,quaternion_errors(:,3),'r');
legend('quaternion error')
xlabel('time');
ylabel('qy');

subplot(4,2,8);
plot(t,quaternion_errors(:,4),'r');
legend('quaternion error')
xlabel('time');
ylabel('qz');