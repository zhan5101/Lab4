function T = ECE569_MatrixExp6(se3mat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a se(3) representation of exponential coordinates.
% Returns a T matrix in SE(3) that is achieved by traveling along/about the 
% screw axis S for a distance theta from an initial configuration T = I.
% Example Input:
% 
% clear; clc;
% se3mat = [ 0,      0,       0,      0;
%          0,      0, -1.5708, 2.3562;
%          0, 1.5708,       0, 2.3562;
%          0,      0,       0,      0]
% T = MatrixExp6(se3mat)
% 
% Output:
% T =
%    1.0000         0         0         0
%         0    0.0000   -1.0000   -0.0000
%         0    1.0000    0.0000    3.0000
%         0         0         0    1.0000 

omgtheta = ECE569_so3ToVec(se3mat(1: 3, 1: 3));
if ECE569_NearZero(norm(omgtheta))
    % T = ...
    T = [eye(3), se3mat(1:3,4); 0 0 0 1];   % 纯平移：v*theta 直接作为平移
else
    [~, theta] = ECE569_AxisAng3(omgtheta);
    omgmat = se3mat(1: 3, 1: 3) / theta; 
    % T = ...
    I3 = eye(3);
    omgmat2 = omgmat * omgmat;

    % Rodrigues 公式得到 R
    R = I3 + sin(theta)*omgmat + (1 - cos(theta))*omgmat2;

    % 左雅可比 V(theta)
    V = I3*theta + (1 - cos(theta))*omgmat + (theta - sin(theta))*omgmat2;

    % 列 4 是 v*theta → 先还原 v，再算 p = V*v
    vtheta = se3mat(1:3,4);
    v = vtheta / theta;
    p = V * v;

    T = [R, p; 0 0 0 1];
end
end
