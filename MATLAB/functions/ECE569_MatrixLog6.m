function se3mat = ECE569_MatrixLog6(T)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Returns the matrix logarithm of a homogeneous transform T in SE(3).
% Output se3mat satisfies: expm(se3mat) = T
% se3mat = [ [omega]theta,  v*theta
%             0  0  0          0     ]

R = T(1:3,1:3);
p = T(1:3,4);
I3 = eye(3);

% 纯平移
if norm(R - I3, 'fro') < 1e-12
    se3mat = [zeros(3), p; 0 0 0 0];
    return;
end

% so(3) 对数： [omega] * theta
tr_arg = (trace(R) - 1) / 2;
tr_arg = max(-1, min(1, tr_arg));
theta = acos(tr_arg);
theta = max(theta, 1e-12);

so3mat   = ECE569_MatrixLog3(R);   % = [omega] * theta
omegaHat = so3mat / theta;

% 构造 V(theta)（左雅可比）
V = I3*theta + (1 - cos(theta)) * omegaHat + (theta - sin(theta)) * (omegaHat*omegaHat);

% 线性方程求 v： V * v = p
v = V \ p;

% 关键：MR 约定的 se3 对数返回的是 ξ̂ θ，因此平移块必须是 v*theta
se3mat = [so3mat, v * theta; 0 0 0 0];
end
