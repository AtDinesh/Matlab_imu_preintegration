function [acc gyro ts] = generateIMU(x0, x1, hx, y0, y1, hy, z0, z1, hz, ox0, ox1, ohx, oy0, oy1, ohy, oz0, oz1, ohz, t0, t1, dt )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
close all;

%% generate X Y Z position functions

cX = fun_generator(x0, x1, t0, t1, hx);
cY = fun_generator(y0, y1, t0, t1, hy);
cZ = fun_generator(z0, z1, t0, t1, hz);

%% plot trajectory (position + velocity + acceleration)
p_ = [];
dp_ = [];
ddp_ = [];

for t=t0:dt:t1
    px = cX(1) + cX(2)*t + cX(3)*t^2 + cX(4)*t^3 + cX(5)*t^4 + cX(6)*t^5 + cX(7)*t^6;
    py = cY(1) + cY(2)*t + cY(3)*t^2 + cY(4)*t^3 + cY(5)*t^4 + cY(6)*t^5 + cY(7)*t^6;
    pz = cZ(1) + cZ(2)*t + cZ(3)*t^2 + cZ(4)*t^3 + cZ(5)*t^4 + cZ(6)*t^5 + cZ(7)*t^6;
    
    dpx = cX(2) + 2*cX(3)*t + 3*cX(4)*t^2 + 4*cX(5)*t^3 + 5*cX(6)*t^4 + 6*cX(7)*t^5;
    dpy = cY(2) + 2*cY(3)*t + 3*cY(4)*t^2 + 4*cY(5)*t^3 + 5*cY(6)*t^4 + 6*cY(7)*t^5;
    dpz = cZ(2) + 2*cZ(3)*t + 3*cZ(4)*t^2 + 4*cZ(5)*t^3 + 5*cZ(6)*t^4 + 6*cZ(7)*t^5;
    
    ddpx =  2*cX(3) + 6*cX(4)*t + 12*cX(5)*t^2 + 20*cX(6)*t^3 + 30*cX(7)*t^4;
    ddpy = 2*cY(3) + 6*cY(4)*t + 12*cY(5)*t^2 + 20*cY(6)*t^3 + 30*cY(7)*t^4;
    ddpz = 2*cZ(3) + 6*cZ(4)*t + 12*cZ(5)*t^2 + 20*cZ(6)*t^3 + 30*cZ(7)*t^4;
    
    p = [px py pz];
    dp = [dpx dpy dpz];
    ddp = [ddpx ddpy ddpz];
    
    p_ = [p_ p'];
    dp_ = [dp_ dp'];
    ddp_ = [ddp_ ddp'];
end

t_ = (t0:dt:t1);

figure('Name','Generated trajectory','NumberTitle','off');
subplot(3,1,1);
plot(t_(1,:), p_(1,:), 'r');
hold on;
plot(t_(1,:), p_(2,:), 'g');
plot(t_(1,:), p_(3,:), 'b');
legend('x', 'y', 'z');
title('position wrt time')

subplot(3,1,2);
plot(t_(1,:), dp_(1,:), 'r');
hold on;
plot(t_(1,:), dp_(2,:), 'g');
plot(t_(1,:), dp_(3,:), 'b');
legend('vx', 'vy', 'vz');
title('velocity wrt time')

subplot(3,1,3);
plot(t_(1,:), ddp_(1,:), 'r');
hold on;
plot(t_(1,:), ddp_(2,:), 'g');
plot(t_(1,:), ddp_(3,:), 'b');
legend('ax', 'ay', 'az');
title('acceleration wrt time')

%% generate WX WY WZ position functions

% cWx = fun_generator(0, 0, t0, t1, 0.5);
% cWy = fun_generator(0, 0, t0, t1, -0.2);
% cWz = fun_generator(0, 0, t0, t1, 0.1);

cWx = fun_generator(ox0, ox1, t0, t1, ohx);
cWy = fun_generator(oy0, oy1, t0, t1, ohy);
cWz = fun_generator(oz0, oz1, t0, t1, ohz);

%% plot trajectory (orientation + angular velocity)
o_ = [];
do_= [];

for t=t0:dt:t1
    ox = cWx(1) + cWx(2)*t + cWx(3)*t^2 + cWx(4)*t^3 + cWx(5)*t^4 + cWx(6)*t^5 + cWx(7)*t^6;
    oy = cWy(1) + cWy(2)*t + cWy(3)*t^2 + cWy(4)*t^3 + cWy(5)*t^4 + cWy(6)*t^5 + cWy(7)*t^6;
    oz = cWz(1) + cWz(2)*t + cWz(3)*t^2 + cWz(4)*t^3 + cWz(5)*t^4 + cWz(6)*t^5 + cWz(7)*t^6;
    
    dox = cWx(2) + 2*cWx(3)*t + 3*cWx(4)*t^2 + 4*cWx(5)*t^3 + 5*cWx(6)*t^4 + 6*cWx(7)*t^5;
    doy = cWy(2) + 2*cWy(3)*t + 3*cWy(4)*t^2 + 4*cWy(5)*t^3 + 5*cWy(6)*t^4 + 6*cWy(7)*t^5;
    doz = cWz(2) + 2*cWz(3)*t + 3*cWz(4)*t^2 + 4*cWz(5)*t^3 + 5*cWz(6)*t^4 + 6*cWz(7)*t^5;
    
    o = [ox oy oz];
    do = [dox doy doz];
    
    o_ = [o_ o'];
    do_ = [do_ do'];
end

figure('Name','Generated orientations','NumberTitle','off');
subplot(2,1,1);
plot(t_(1,:), o_(1,:), 'r');
hold on;
plot(t_(1,:), o_(2,:), 'g');
plot(t_(1,:), o_(3,:), 'b');
legend('ox', 'oy', 'oz');
title('orientation wrt time')

subplot(2,1,2);
plot(t_(1,:), do_(1,:), 'r');
hold on;
plot(t_(1,:), do_(2,:), 'g');
plot(t_(1,:), do_(3,:), 'b');
legend('wx', 'wy', 'wz');
title('angular velocity wrt time')

%% outputs

acc = ddp_;
gyro = do_;
ts = t_;

end

