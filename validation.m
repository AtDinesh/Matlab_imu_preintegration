clear all;
close all;

%% VALIDATION TESTS

%% data2delta validation
% bias
syms bax bay baz bwx bwy bwz real;
b = [bax;bay;baz;bwx;bwy;bwz];

% measurement
syms ax ay az wx wy wz dt real;
u = [ax;ay;az;wx;wy;wz];

n_disc = rand(6,1); % bias random walk in discrete time

[d, D_b, D_u, D_nd] = data2delta(b, u, n_disc, dt)

%% deltaPlusDelta validation
%PArams:
%In:
%  di: integrated delta size 10 
%  d : instantaneous delta
%Out:
%  di_out: integrated delta

syms dt real;
syms dipx dipy dipz diqw diqx diqy diqz divx divy divz real;
syms dpx dpy dpz dqw dqx dqy dqz dvx dvy dvz real;
di = [dipx;dipy;dipz;diqw;diqx;diqy;diqz;divx;divy;divz];
d = [dpx;dpy;dpz;dqw;dqx;dqy;dqz;dvx;dvy;dvz];
[di_out, DI_OUT_di, DI_OUT_d] = deltaPlusDelta (di, d, dt)

%delta ranges
pr = 1:3;
qr = 4:7;
vr = 8:10;
[di_out, DI_OUT_di, DI_OUT_d] = deltaPlusDelta (di, d, dt)


%% deltaMinusDelta validation
% %[ di_out, DI_OUT_di, DI_OUT_d ] = deltaMinusDelta( di, d, dt )
% 
% %let us first test with same deltas to make sure the method is correct
% syms dt real;
% syms dipx dipy dipz diqw diqx diqy diqz divx divy divz real;
% di = [dipx;dipy;dipz;diqw;diqx;diqy;diqz;divx;divy;divz];
% d=di;
% 
% di_out = deltaMinusDelta2(di, d, dt)
% %validation numerique
% disp('validation numerique')
% di = rand(10,1)
% d=di;
% dt=0.01;
% di_out = deltaMinusDelta2(di, d, dt)

%% deltaMinusDelta validation
%[ di_out, DI_OUT_di, DI_OUT_d ] = deltaMinusDelta( di, d, dt )

%let us first test with same deltas to make sure the method is correct
% syms dt real;
% syms dipx dipy dipz diqw diqx diqy diqz divx divy divz real;
% di = [dipx;dipy;dipz;diqw;diqx;diqy;diqz;divx;divy;divz];
% d=di;
% di_out = deltaMinusDelta2(di, d, dt)

%% unitary tests

disp('first tests');
q = rand(1,4)
q = quatnormalize(q)
qc = q2qc(q)
u = rand(6,1);
b = rand(6,1)/100;
n_disc = rand(6,1)/100; % bias random walk in discrete time
dt=0.01;
[d, D_b, D_u, D_nd] = data2delta(b, u, n_disc, dt)
dv = d(8:10);

% [dv_tmp, DVT_dv, DVT_dqi_out] =  qRot(dv, dqi_out);
[dv_tmp1, DVT_dv, DVT_dqi_out] =  qRot(dv, q);
[dv_tmp, DVT_dv, DVT_dqi_out] =  qRot(dv, qc);
dv_tmp
dv_tmp1

% %validation numerique
% disp('validation numerique')
% u = rand(6,1);
% b = rand(6,1)/100;
% n_disc = rand(6,1)/100; % bias random walk in discrete time
% dt=0.01;
% disp('data2Delta')
% [d, D_b, D_u, D_nd] = data2delta(b, u, n_disc, dt);
% d
% di = [0; 0 ;0; 1; 0; 0; 0 ;0; 0; 0]
% % disp('deltaPlusDelta')
% % [di_out, DI_OUT_di, DI_OUT_d] = deltaPlusDelta (di, d, dt)
% %example for delta : delta1 = [0.0041; -0.0077;-0.0019;0.9014;-0.1123;-0.4073;-0.0944;0.2752;-0.5156;-0.1237]
% % other example : delta0 = [0.0019;-0.0061;-0.0063;0.9014;-0.1123;-0.4073;-0.0944;0.1262;-0.4058;-0.4197]
% 
% di_out = [0.0019;-0.0061;-0.0063;0.9014;-0.1123;-0.4073;-0.0944;0.1262;-0.4058;-0.4197]
% di_out = deltaMinusDelta2(di_out, di_out, dt)
%% COMPLETE VALIDATION

% syms ax ay az wx wy wz abx aby abz wbx wby wbz dt real;
% syms dipx dipy dipz diqw diqx diqy diqz divx divy divz real;
% u = [ax;ay;az;wx;wy;wz];
% di = [dipx;dipy;dipz;diqw;diqx;diqy;diqz;divx;divy;divz];
% b = [abx;aby;abz;wbx;wby;wbz];
% % u = rand(6,1);
% % di = rand(10,1);
% % b = rand(6,1);
% syms anx any anz wnx wny wnz real;
% n = [anx;any;anz;wnx;wny;wnz];
% %n = rand(6,1);
% syms dt real;
% %dt = 0.001;
% disp('Entering integrateDelta\n')
% [Di_out, DI_OUT_di, DI_OUT_b, DI_OUT_u, DI_OUT_n] = integrateDelta (di, b, u, n, dt);
% disp('Succedded integrateDelta\n')
% disp('Computing Jacobian DI_OUT_b_sym')
% DI_OUT_b_sym = jacobian(di_out, b);
% disp('Computing residual')
% DI_OUT_b_test = simplify(DI_OUT_b - DI_OUT_b_sym)