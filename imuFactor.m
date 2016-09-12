m% ## Copyright (C) 2016 Joan Solà
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
% ## @deftypefn {Function File} {@var{retval} =} imuFactor (@var{input1}, @var{input2})
% ##
% ## @seealso{}
% ## @end deftypefn
% 
% ## Author: Joan Solà <jsola@dhcp-9-225.laas.fr>
% ## Created: 2016-04-28
% 

function [r, R_xi, R_xj, R_gz] = imuFactor (xi, xj, dt, D_b)

g = [0;0;-9.8];

% We need to have bi0 stored: bi0 is the value of the bias we used to compute dhte pre-integrated delta, that is, it corresponds to time t_i
% CHeck that this is OK with the paper

% delta ranges
pr = 1:3;
qr = 4:7;
vr = 8:10;
abr = 11:13;
wbr = 14:16;

% split states
pi = xi(pr);
qi = xi(qr);
vi = xi(vr);
abi = xi(abr);
wbi = xi(wbr);
bi = [abi, wbi];

pj = xj(pr);
qj = xj(qr);
vj = xj(vr);
abj = xj(abr);
wbj = xj(wbr);
bj = [abj, wbj];

% Delta prediction from the state
qic = q2qc(qi);
dq = qProd(qic,qj); % qi' * qj;
[dv, DV_qic, DV_vj] = qRot(qic, vj - vi - g*dt);
DV_vi = -DV_vj;
DV_gz = DV_vi(:,3)*dt; % In case we want to calibrate the Z component of g.
dp = qRot(qic, pj - pi  - vi*dt - 0.5*g*dt^2);

d_pred = [dp, dq, dv];

% Correction of the pre-integrated delta from the bias viariation
%d_int_pred = d + D_b*(bi - bi0);

% Residual
%r = deltaMinusDelta(d_pred, d + D_b*(bj-bi), dt); % Get inspired from deltaPlusDelta()

end
