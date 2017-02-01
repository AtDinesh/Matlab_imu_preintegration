% ## Copyright (C) 2016 Joan Solà
% ## 
% ## This program is free software; you can redistribute it and/or modify it
% ## under the terms of the GNU General Public License as published by
% ## the Free Software Foundation; either version 3 of the License, or
% ## (at your option) any later version.
% ## 
% ## This program is distributed in the hope that it will be useful,
% ## but WITHOUT ANY WARRANTY; without even the implied warranty of
% ## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% ## GNU General Public License for more details.
% ## 
% ## You should have received a copy of the GNU General Public License
% ## along with this program.  If not, see <http://www.gnu.org/licenses/>.
% 
% ## -*- texinfo -*- 
% ## @deftypefn {Function File} {@var{retval} =} deltaPlusDelta (@var{input1}, @var{input2})
% ##
% ## @seealso{}
% ## @end deftypefn
% 
% ## Author: Joan Solà <jsola@dhcp-9-225.laas.fr>
% ## Created: 2016-04-28

function [di_out, DI_OUT_di, DI_OUT_d] = deltaPlusDelta (di, d, dt)

% PArams:
% In:
%   di: integrated delta
%   d : instantaneous delta
% Out:
%   di_out: integrated delta

%EQUATIONS
% dp_out = dpi + dvi*dt + 0.5*qRot(a*dt2, dqi)
% dv_out = dvi + qRot(a*dt, dqi)
% dq_out = dqi * Exp(w*dt)

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
[dqi_out, DQI_OUT_dqi, DQI_OUT_dq] = qProd(dqi,dq);


% velocity integration
[dv_tmp, DVT_dv, DVT_dqi] =  qRot(dv, dqi);
dvi_out = dvi + dv_tmp;
if nargout > 2
    DVI_OUT_dvi = 1;
    DVI_OUT_dvt = 1;
    DVI_OUT_dqi = DVI_OUT_dvt * DVT_dqi;
    DVI_OUT_dv  = DVI_OUT_dvt * DVT_dv;
end

% position integration
[dp_tmp, DPT_dp, DPT_dqi] = qRot(dp, dqi);
dpi_out = dpi + dvi * dt + dp_tmp;
if nargout > 2
    DPI_OUT_dpt = 1;
    DPI_OUT_dpi = 1;
    DPI_OUT_dvi = dt;
    DPI_OUT_dqi = DPI_OUT_dpt * DPT_dqi;
    DPI_OUT_dv  = zeros(3,3);
end

% assemble final delta integrated
di_out = [dpi_out; dqi_out; dvi_out]; % In this order please!

if nargout > 2
    % Jacobian wrt di
    DI_OUT_di = zeros(10,10);
    DI_OUT_di = vpa(DI_OUT_di); %must be removed
    DI_OUT_di(qr,qr) = DQI_OUT_dqi;
    
    % DI_OUT_di(vr,qr) = DVI_OUT_dqi_out * DQI_OUT_dqi;
    DI_OUT_di(vr,qr) = DVI_OUT_dqi;
    DI_OUT_di(vr,vr) = DVI_OUT_dvi;
    
    % DI_OUT_di(pr,qr) = DPI_OUT_dqi_out * DQI_OUT_dqi;
    DI_OUT_di(pr,qr) = DPI_OUT_dqi;
    DI_OUT_di(pr,vr) = DPI_OUT_dvi;
    DI_OUT_di(pr,pr) = DPI_OUT_dpi;
    
    
    % Jacobian wrt d
    DI_OUT_d = zeros(10,10);
    DI_OUT_d = vpa(DI_OUT_d); %must be removed
    DI_OUT_d(qr,qr) = DQI_OUT_dq;
    
    DI_OUT_d(vr,qr) = zeros(3,4);
    DI_OUT_d(vr,vr) = DVI_OUT_dv;
    
    DI_OUT_d(pr,qr) = zeros(3,4);
    DI_OUT_d(pr,vr) = DPI_OUT_dv;
end
end
