function V = ECE569_se3ToVec(se3mat)
% *** CHAPTER 3: RIGID-BODY MOTIONS ***
% Takes a 4x4 se(3) matrix and returns the corresponding 6-vector.
% The output is of the form [omega; v], where omega is the angular part.

% V = ... TODO
w_hat = se3mat(1:3,1:3);
v     = se3mat(1:3,4);

% vee operator for so(3): [w]x -> w
omega = [ w_hat(3,2);
          w_hat(1,3);
          w_hat(2,1) ];

V = [omega; v];
end
