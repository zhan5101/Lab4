function [thetalist, success] = ECE569_IKinBody(Blist, M, Tsd, thetalist0, eomg, ev)
% Inverse Kinematics in the BODY frame (MR, Sec. 6.2; Eq. 6.54+)
% Inputs:
%   Blist      : 6xn body screw axes at home, columns are Bi
%   M          : 4x4 home configuration of the end-effector
%   Tsd        : 4x4 desired configuration of end-effector (in {s})
%   thetalist0 : nx1 initial guess
%   eomg       : orientation error tolerance  (||ω_b|| <= eomg)
%   ev         : position error tolerance     (||v_b|| <= ev)
% Outputs:
%   thetalist  : solution joint angles
%   success    : true(1) if both tolerances are met before max iters

thetalist = thetalist0(:);          % ensure column vector
maxiter   = 100;                    % reasonable default
i         = 0;

% current forward pose
Tsb = ECE569_FKinBody(M, Blist, thetalist);

% body-frame error transform Tbd = inv(Tsb) * Tsd
Tbd = ECE569_TransInv(Tsb) * Tsd;
Vb  = ECE569_se3ToVec(ECE569_MatrixLog6(Tbd));   % [ω_b; v_b]

werr = norm(Vb(1:3));
verr = norm(Vb(4:6));

while (werr > eomg || verr > ev) && i < maxiter
    i  = i + 1;

    % body Jacobian at current thetas
    Jb = ECE569_JacobianBody(Blist, thetalist);

    % damped least-squares / pseudoinverse step (plain pinv is fine here)
    thetalist = thetalist + pinv(Jb) * Vb;

    % update pose and error
    Tsb = ECE569_FKinBody(M, Blist, thetalist);
    Tbd = ECE569_TransInv(Tsb) * Tsd;
    Vb  = ECE569_se3ToVec(ECE569_MatrixLog6(Tbd));

    werr = norm(Vb(1:3));
    verr = norm(Vb(4:6));
end

success = (werr <= eomg) && (verr <= ev);
end
