% ## Copyright (C) 2017 Dinesh Atchuthan
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
% ## @deftypefn {Function File} {@var{retval} =} xPlusDelta (@var{input1}, @var{input2})
% ##
% ## @seealso{}
% ## @end deftypefn
% 
% ## Author: Dinesh Atchuthan <dinesh.atchuthan@laas.fr>
% ## Created: 2017-02-07

function [ x_out ] = xPlusDelta(x, di, Dt)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Params:
% In:
%   x: previous state
%   d : integrated delta
% Out:
%   x_out: updated state

%EQUATIONS
%xp_out_ = xq * Dp_ + xp_ + xv_ * Dt + gDt * Dt / 2 ;
%xv_out_ = xq * Dv_ + xv_ + gDt;
%xq_out_ = xq * Dq;

% delta ranges
pr = 1:3;
qr = 4:7;
vr = 8:10;

dpi = di(pr);
dqi = di(qr);
dvi = di(vr);

xp = x(pr);
xq = x(qr);
xv = x(vr);

g = [0; 0; -9.8060];

% quaternion integration
[xq_out, X_OUT_xq, X_OUT_dqi] = qProd(xq, dqi);

% velocity integration
[xv_tmp, XVT_dvi, XVT_xq] =  qRot(dvi, xq);
xv_out = xv + xv_tmp + g * Dt;

% position integration
[xp_tmp, XPT_dpi, XPT_xq] = qRot(dpi, xq);
xp_out = xp + xv * Dt + xp_tmp + g * (Dt * Dt) / 2;

% assemble final state
x_out = [xp_out; xq_out; xv_out]; % In this order please!

end

