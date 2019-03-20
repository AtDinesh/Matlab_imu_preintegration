clear all;
close all;

%data = load('./tests/save_gtest_CTRIMU_bias_MoveNonNullBiasFreeFalling_VarB1B2.dat');
%data = load('./tests/save_gtest_CTRIMU_bias_NonNullBiasAccCst_VarB1B2.dat');
%data = load('./tests/save_gtest_CTRIMU_bias_TRNonNullBiasRotCst_VarB1B2.dat');
%data = load('./tests/save_gtest_CTRIMU_bias_TRNonNullBiasRotVCst_VarB1B2.dat');
data = load('./tests/save_gtest_CTRIMU_bias_Complex_VarB1B2.dat');


epsilon = data(:,1);
expected_bias = data(:,2:7);
initial_bias = data(:,8:13);
estimated_bias_KF0 = data(:,14:19);
estimated_bias_KF1 = data(:,20:25);

figure('Name','error in estimated bias','NumberTitle','off');
subplot(2,1,1);
plot(epsilon, estimated_bias_KF0(:,1) - expected_bias(:,1), 'b');
hold on;
plot(epsilon, estimated_bias_KF0(:,2) - expected_bias(:,2), 'g');
plot(epsilon, estimated_bias_KF0(:,3) - expected_bias(:,3), 'r');
plot(epsilon, estimated_bias_KF0(:,4) - expected_bias(:,4), 'b--');
plot(epsilon, estimated_bias_KF0(:,5) - expected_bias(:,5), 'g--');
plot(epsilon, estimated_bias_KF0(:,6) - expected_bias(:,6), 'r--');
xlabel('Error in Abx initial guess');
ylabel('KF0 bias estimation error');
legend('Abx', 'Aby', 'Abz', 'Wbx', 'Wby', 'Wbz');

subplot(2,1,2);
plot(epsilon, estimated_bias_KF1(:,1) - expected_bias(:,1), 'b');
hold on;
plot(epsilon, estimated_bias_KF1(:,2) - expected_bias(:,2), 'g');
plot(epsilon, estimated_bias_KF1(:,3) - expected_bias(:,3), 'r');
plot(epsilon, estimated_bias_KF1(:,4) - expected_bias(:,4), 'b--');
plot(epsilon, estimated_bias_KF1(:,5) - expected_bias(:,5), 'g--');
plot(epsilon, estimated_bias_KF1(:,6) - expected_bias(:,6), 'r--');
xlabel('Error in Abx initial guess');
ylabel('KF1 bias estimation error');
legend('Abx', 'Aby', 'Abz', 'Wbx', 'Wby', 'Wbz');

evol_KF0_abx = [initial_bias(:,1)'; estimated_bias_KF0(:,1)'];
evol_KF0_aby = [initial_bias(:,2)'; estimated_bias_KF0(:,2)'];
evol_KF0_abz = [initial_bias(:,3)'; estimated_bias_KF0(:,3)'];

evol_KF0_wbx = [initial_bias(:,4)'; estimated_bias_KF0(:,4)'];
evol_KF0_wby = [initial_bias(:,5)'; estimated_bias_KF0(:,5)'];
evol_KF0_wbz = [initial_bias(:,6)'; estimated_bias_KF0(:,6)'];


figure('Name','evolution of acc bias','NumberTitle','off');
subplot(3,1,1);
plot(evol_KF0_abx);
ylabel('abx');
xlabel('1:initial, 2:optimized');
subplot(3,1,2);
plot(evol_KF0_aby);
ylabel('aby');
xlabel('1:initial, 2:optimized');
subplot(3,1,3);
plot(evol_KF0_abz);
ylabel('abz');
xlabel('1:initial, 2:optimized');

figure('Name','evolution of gyro bias','NumberTitle','off');
subplot(3,1,1);
plot(evol_KF0_wbx);
ylabel('wbx');
xlabel('1:initial, 2:optimized');
subplot(3,1,2);
plot(evol_KF0_wby);
ylabel('wby');
xlabel('1:initial, 2:optimized');
subplot(3,1,3);
plot(evol_KF0_wbz);
ylabel('wbz');
xlabel('1:initial, 2:optimized');
