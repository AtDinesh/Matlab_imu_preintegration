function [ H ] = homogenize(R)
syms r03 r13 r23 r33 r30 r31 r32 real 

if size(R,1) == 3
    H = [ R(1,:), r03; R(2,:), r13; R(3,:), r23; r30,r31, r32,r33]

end

return
end