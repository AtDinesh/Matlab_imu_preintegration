function [ q_out ] = quatnormalize( q )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if size(q,2) == 4
    q_out = q/(sqrt(q(1)*q(1) + q(2)*q(2) + q(3)*q(3) + q(4)*q(4)));
else
    error('quaternions must be of size 4, q = qw + i*qx + j*qy + k*qz');

end