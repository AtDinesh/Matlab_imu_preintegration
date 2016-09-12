function [ di_out, DI_OUT_di, DI_OUT_d ] = deltaMinusDelta2( di, d, dt )
%deltaMinusDelta Summary of this function goes here
%   Detailed explanation goes here

% PArams:
% In:
%   di: integrated delta we substract from
%   d : integrated delta we wihs to substract
% Out:
%   di_out: residual integrated delta


%% delta ranges
pr = 1:3;
qr = 4:7;
vr = 8:10;

dpi = di(pr);
dqi = di(qr);
dvi = di(vr);

dp = d(pr);
dq = d(qr);
dv = d(vr);

%% careful : manifold operation is OK for small rotations...
%% quaternion integration
dqc = q2qc(dq);
[dqi_out, DQI_OUT_dqi, DQI_OUT_dq] = qProd(dqi,dqc); %change here


%% velocity integration
%rotation invere deja prise en compte dans le conjugue
[dv_tmp, DVT_dv, DVT_dqi_out] =  qRot(dv,dqc);
% dvi_out = dvi + dv_tmp;
dvi_out = dvi - dv_tmp;

DVI_OUT_dvi = 1;
DVI_OUT_dvt = 1;
%DVI_OUT_dqi = DVI_OUT_dvt * DVT_dqi_out * DQI_OUT_dqi;
%DVI_OUT_dqi = DVT_dqi_out * DQI_OUT_dqi; %not working and not used
DVI_OUT_dv  = DVI_OUT_dvt * DVT_dv;

%% position integration
% dpi_out = dpi + 1.5 * dv_tmp * dt;
dpi_out = dpi - 1.5 * dv_tmp * dt;
DPI_OUT_dpi = 1;
DPI_OUT_dvt = 1.5 * dt;
DPI_OUT_dqi = DPI_OUT_dvt * DVT_dqi_out * DQI_OUT_dqi; 
DPI_OUT_dv  = DPI_OUT_dvt * DVT_dv;

%% assemble final delta integrated
di_out = [dpi_out; dqi_out; dvi_out];

%% Jacobians
% Jacobian wrt di
DI_OUT_di = zeros(10,10);
DI_OUT_di = vpa(DI_OUT_di); %must be removed
DI_OUT_di(qr,qr) = DQI_OUT_dqi;

%DI_OUT_di(vr,qr) = DVI_OUT_dqi_out * DQI_OUT_dqi;
%above equivalent to :
DI_OUT_di(vr,qr) = DVT_dqi_out * DQI_OUT_dqi;
DI_OUT_di(vr,vr) = DVI_OUT_dvi;

DI_OUT_di(pr,qr) = DPI_OUT_dqi;
%DI_OUT_di(pr,vr) = DPI_OUT_dvt * DVT_dvi;
%DVT_dvi = 0;
DI_OUT_di(pr,pr) = DPI_OUT_dpi;

% Jacobian wrt d
DI_OUT_d = zeros(10,10);
DI_OUT_d = vpa(DI_OUT_d); %must be removed
DI_OUT_d(qr,qr) = DQI_OUT_dq;

DI_OUT_d(vr,qr) = DVI_OUT_dvt * DVT_dqi_out * DQI_OUT_dq;
DI_OUT_d(vr,vr) = DVI_OUT_dv;

DI_OUT_d(pr,qr) = DPI_OUT_dvt * DVT_dqi_out * DQI_OUT_dq;
DI_OUT_d(pr,vr) = DPI_OUT_dv;

end

