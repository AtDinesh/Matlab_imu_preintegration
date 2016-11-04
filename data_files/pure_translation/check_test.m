clear all;
close all;

%load files
test = load('0/debug_results.dat'); %without correction of inital state
check_data = load('0/data_pure_translation_check.txt'); % in PQV formulation

%ranges
delta_range = 2:11;
Delta_range = 12:21; %File in PVQ
State_range = 22:31;
cpp_qr = 7:10;
matlab_qr = 4:7;
cpp_pr = 1:3;
cpp_vr = 4:6;
mat_pr = 2:4;
or = 5:7;

%get data
test_delta = test(:,delta_range);
test_Delta = test(:,Delta_range);
test_State = test(:,State_range);

ref_pos = check_data(:,mat_pr);
ref_or = check_data(:,or);

cpp_pos_delta = [];
cpp_vel_delta = [];
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
    Dq_ = test_Delta(i,cpp_qr);
    Dq = [Dq_(1,4) Dq_(1,1) Dq_(1,2) Dq_(1,3)];
    Xq_ = test_State(i,cpp_qr);
    Xq = [Xq_(1,4) Xq_(1,1) Xq_(1,2) Xq_(1,3)];
    cpp_or_Delta = [cpp_or_Delta q2v(Dq')];
    cpp_or_State = [cpp_or_State q2v(Xq')];
end

cpp_pos_delta = test_delta(:,cpp_pr);
cpp_vel_Delta = test_delta(:,cpp_vr);
cpp_pos_Delta = test_Delta(:,cpp_pr);
cpp_pos_State = test_State(:,cpp_pr);
cpp_vel_Delta = test_Delta(:,cpp_vr);
t = check_data(:,1);

%% PLOTS

%position
figure('Name','3D position plot','NumberTitle','off');
plot3(check_data(:,2),check_data(:,3),check_data(:,4), 'g');
hold on;
plot3(cpp_pos_Delta(:,1), cpp_pos_Delta(:,2), cpp_pos_Delta(:,3), 'r');
plot3(cpp_pos_State(:,1), cpp_pos_State(:,2), cpp_pos_State(:,3), 'm');
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('real Position State', 'integrated Delta Position', 'Integrated State Position');



%position
figure('Name','3D position plot','NumberTitle','off');
plot3(check_data(:,2),check_data(:,3),check_data(:,4), 'g');
hold on;
xlabel('x posititon');
ylabel('y posititon');
zlabel('z posititon');
legend('real Position State');

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

%position deltas
figure('Name','position through time','NumberTitle','off');
subplot(3,1,1);
plot(t, cpp_pos_delta(:,1),'r');
xlabel('time');
ylabel('position x delta');
legend('instantaneous delta position x');

subplot(3,1,2);
plot(t, cpp_pos_delta(:,2),'r');
xlabel('time');
ylabel('position x delta');
legend('instantaneous delta position y');

subplot(3,1,3);
plot(t, cpp_pos_delta(:,3),'r');
xlabel('time');
ylabel('position x delta');
legend('instantaneous delta position z');

%Velocity
figure('Name','Velocity through time','NumberTitle','off');
subplot(3,1,1);
plot(t, cpp_vel_Delta(:,1), 'g');
hold on;
xlabel('time');
ylabel('velocity X');
legend('Integrated Velocity');

subplot(3,1,2);
plot(t, cpp_vel_Delta(:,2), 'g');
hold on;
xlabel('time');
ylabel('velocity Y');
legend('Integrated Velocity');

subplot(3,1,3);
plot(t, cpp_vel_Delta(:,3), 'g');
hold on;
xlabel('time');
ylabel('velocity Z');
legend('Integrated Velocity');

% %orientation
% figure('Name','orientation through time','NumberTitle','off');
% subplot(3,1,1);
% plot(t, check_data(:,5), 'g');
% hold on;
% plot(t, cpp_or_Delta(1,:),'r');
% xlabel('time');
% ylabel('angle x');
% legend('Real orientation state', 'integrated Delta orientation state');
% hold on;
% subplot(3,1,2);
% plot(t, check_data(:,6), 'g');
% hold on;
% plot(t, cpp_or_Delta(2,:),'r');
% xlabel('time');
% ylabel('angle y');
% legend('reconstructed orientation state', 'integrated Delta orientation state');
% subplot(3,1,3);
% plot(t, check_data(:,7), 'g');
% hold on;
% plot(t, cpp_or_Delta(3,:),'r');
% xlabel('time');
% ylabel('angle z');
% legend('reconstructed orientation state', 'integrated Delta orientation state');