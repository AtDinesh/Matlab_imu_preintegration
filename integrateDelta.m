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
% ## @deftypefn {Function File} {@var{retval} =} integrateDelta (@var{input1}, @var{input2})
% ##
% ## @seealso{}
% ## @end deftypefn
% 
% ## Author: Joan Solà <jsola@dhcp-9-225.laas.fr>
% ## Created: 2016-04-28

function [di_out, DI_OUT_di, DI_OUT_b, DI_OUT_u, DI_OUT_n, d] = integrateDelta (di, b, u, n, dt)

if nargout == 1
  d = data2delta(b, u, n, dt);
  di_out = deltaPlusDelta(di, d, dt);
else
  %disp('Entering data2delta (multiple output)\n')
  [d, D_b, D_u, D_n] = data2delta(b, u, n, dt);
  %disp('Succedded data2delta (multiple output)\n')
  %disp('Entering deltaPlusDelta (multiple output)\n')
  [di_out, DI_OUT_di, DI_OUT_d] = deltaPlusDelta(di, d, dt);
  %disp('Succedded deltaPlusDelta (multiple output)\n')
  
  %disp('Let us apply the Chain rule\n')
  % Chain rule
  DI_OUT_b = DI_OUT_d * D_b;
  DI_OUT_u = DI_OUT_d * D_u;
  DI_OUT_n = DI_OUT_d * D_n;
  %disp('Succedded Chain rule operation\n')

%  if nargout == 5  
    % Covariances
%    DI_OUT = DI_OUT_b * B * DI_OUT_b' + DI_OUT_di * DI * DI_OUT_di' + DI_OUT_n * N * DI_OUT_n'; % + DI_OUT_u * U * DI_OUT_u'
%  end
  
end  




end

%% 
% syms ax ay az wx wy wz abx aby abz wbx wby wbz dt real
% syms dipx dipy dipz diqw diqx diqy diqz divx divy divz;
% u = [ax;ay;az;wx;wy;wz];
% di = [dipx;dipy;dipz;diqw;diqx;diqy;diqz;divx;divy;divz];
% b = [abx;aby;abz;wbx;wby;wbz];
% n = rand(6,1)
% dt = 0.001;
% [di_out, DI_OUT_di, DI_OUT_b, DI_OUT_u, DI_OUT_n] = integrateDelta (di, b, u, n, dt);
% DI_OUT_b_sym = jacobian(di_out, b);
% DI_OUT_b_test = simplify(DI_OUT_b - DI_OUT_b_sym)