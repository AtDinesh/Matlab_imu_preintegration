syms ax ay az wx wy wz abx aby abz wbx wby wbz dt real;
syms dipx dipy dipz diqw diqx diqy diqz divx divy divz real;
u = [ax;ay;az;wx;wy;wz];
di = [dipx;dipy;dipz;diqw;diqx;diqy;diqz;divx;divy;divz];
b = [abx;aby;abz;wbx;wby;wbz];
% u = rand(6,1);
% di = rand(10,1);
% b = rand(6,1);
syms anx any anz wnx wny wnz real;
n = [anx;any;anz;wnx;wny;wnz];
%n = rand(6,1);
syms dt real;
%dt = 0.001;
[di_out, DI_OUT_di, DI_OUT_b, DI_OUT_u, DI_OUT_n] = integrateDelta (di, b, u, n, dt);
DI_OUT_b_sym = jacobian(di_out, b);
DI_OUT_b_test = simplify(DI_OUT_b - DI_OUT_b_sym)