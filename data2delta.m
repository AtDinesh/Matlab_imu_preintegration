function [d, D_b, D_u, D_nd] = data2delta(b, u, n_disc, dt)

%% Params
% In:
%   b: bias vector 6x1
%   u: measurement vector 6x1
%   n_disc: bias random walk in discrete time
%   dt: smapling time
% Out:
%   d: instantaneous delta 10x1
%   D_b jac of d wrt b
%   D_u: jac of d wrt u
%   D_nd: jac wrt n_disc

% split measurements
am = u(1:3);
wm = u(4:6);

% split bias
ab = b(1:3);
wb = b(4:6);

% split bias perturbations
an = n_disc(1:3);
wn = n_disc(4:6);

% delta ranges
pr = 1:3;
qr = 4:7;
vr = 8:10;

% measurement, bias and noise ranges
ar = 1:3;
wr = 4:6;

%% REFER TO SOLA-16 for equations
%v_out_ = a * _dt;
%p_out_ = v_out_ * _dt / 2;
%q_out_ = v2q(w * _dt);

%% Euler integration (zero-order integration) for inputs, non-integration for perturbation (perturbation is already integrated)
adt = (am - ab)*dt - an;
wdt = (wm - wb)*dt - wn;
ADT_am = dt;
WDT_wm = dt;
ADT_ab = -dt;
WDT_wb = -dt;
ADT_an = -1;
WDT_wn = -1;

%% Exponential map
[dq, DQ_wdt] = v2q(wdt);  % this is exp(wdt)

% projection onto the manifold
dv = a*dt;
DV_adt = 1;
DV_dq = 0;
dp = 1.5*dv*dt;
DP_dv = 1.5*dt;
DP_adt = DP_dv*DV_adt;

% Compose final delta vector
d = [dp ; dq ; dv]; % This order, please!

%DV_dq = homogenize(DV_dq);
%% Jacobian wrt bias perturbation in discrete time
DQ_wn = DQ_wdt * WDT_wn;
DV_an = DV_adt * ADT_an;
DV_wn = DV_dq  * DQ_wn;
DP_an = DP_adt * ADT_an;
DP_wn = DP_dv  * DV_wn;
D_nd = zeros(10,6);
%D_nd = vpa(D_nd); %must be removed
% D_nd(qr, wr) = DV_wn;
% D_nd(vr, ar) = DQ_wn;
% D_nd(vr, wr) = DV_an;
% D_nd(pr, ar) = DP_an;
% D_nd(pr, wr) = DP_wn;

D_nd(qr, wr) = DQ_wn;
D_nd(vr, ar) = DV_an;
D_nd(vr, wr) = DV_wn;
D_nd(pr, ar) = DP_an;
D_nd(pr, wr) = DP_wn;

%% Jacobian wrt bias

%DQ_wb = DQ_wdt * WDT_wb;
%DV_ab = DV_adt * ADT_ab;
%DV_wb = DV_dq  * DQ_wb;
%DP_ab = DP_adt * ADT_ab;
%DP_wb = DP_dv  * DV_wb;

%D_b = zeros(10,6);
%D_b(qr, wr) = DV_wb;
%D_b(vr, ar) = DQ_wb;
%D_b(vr, wr) = DV_ab;
%D_b(pr, ar) = DP_ab;
%D_b(pr, wr) = DP_wb;

% This boils down to:
D_b = D_nd * dt;

%% Jacobian wrt measurement
D_u = -D_b; % we are not going to use it because the measurement is deterministic

end

%%%%
%% Validation

% % bias
% syms bax bay baz bwx bwy bwz real;
% b = [bax;bay;baz;bwx;bwy;bwz];
% 
% % measurement
% syms ax ay az wx wy wz dt real;
% u = [ax;ay;az;wx;wy;wz];
% 
% n_disc = rand(6,1); % bias random walk in discrete time
% 
% [d, D_b, D_u, D_nd] = data2delta(b, u, n_disc, dt);
