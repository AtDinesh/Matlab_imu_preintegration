clear all;
close all;

data = load('./tests/save_gtest_CTRIMU_bias_MoveNonNullBiasFreeFalling_VarB1B2.dat');

epsilon = data(:,1);
expected_bias = data(:,2:7);
estimated_bias_KF0 = data(:,8:13);
estimated_bias_KF1 = data(:,14:19);

figure('Name','position through time','NumberTitle','off');
subplot(2,1,1);
plot(epsilon, estimated_bias_KF0(:,1) - expected_bias(:,1), 'b');
hold on;
plot(epsilon, estimated_bias_KF0(:,2) - expected_bias(:,2), 'g');
plot(epsilon, estimated_bias_KF0(:,3) - expected_bias(:,3), 'r');
plot(epsilon, estimated_bias_KF0(:,4) - expected_bias(:,4), 'b--');
plot(epsilon, estimated_bias_KF0(:,5) - expected_bias(:,5), 'g--');
plot(epsilon, estimated_bias_KF0(:,6) - expected_bias(:,6), 'r--');
xlabel('Error in initial guess');
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
xlabel('Error in initial guess');
ylabel('KF1 bias estimation error');
legend('Abx', 'Aby', 'Abz', 'Wbx', 'Wby', 'Wbz');