function [di_out] = imu_integrator (di, d, dt)

% delta ranges
pr = 1:3;
qr = 4:7;
vr = 8:10;

dpi = di(pr);
dqi = di(qr);
dvi = di(vr);

dp = d(pr);
dq = d(qr);
dv = d(vr);

% quaternion integration
[dqi_out] = qProd(dqi,dq); % dq_out = dqi*dq

%velocity integration
[dv_tmp] =  qRot(dv, dqi); %dv = adt
dvi_out = dvi + dv_tmp; % dv_out = dvi + dqi * dv

%position integration
[dp_tmp] = qRot(dp, dqi); %dp = 0.5 * a * dt2
dpi_out = dpi + dvi * dt + dp_tmp; % dp_out = dpi + dvi*dt + dqi * dp

% assemble final delta integrated
di_out = [dpi_out; dqi_out; dvi_out];

end