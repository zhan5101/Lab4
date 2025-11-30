function Jb = ECE569_JacobianBody(Blist, thetalist)
% *** CHAPTER 5: VELOCITY KINEMATICS AND STATICS ***

Jb = Blist;          % 最后一列 Jb(:,n) = Bn 已就位
T  = eye(4);         % 累积 Ti = e^{-B_{i+1}θ_{i+1}} ... e^{-B_n θ_n}

for i = length(thetalist) - 1 : -1 : 1
    % 累积 Ti ← Ti * e^{-B_{i+1} θ_{i+1}}
    T = T * ECE569_MatrixExp6( ECE569_VecTose3( -Blist(:, i+1) * thetalist(i+1) ) );

    % Jb(:,i) = Ad_{Ti} * Bi
    Jb(:, i) = ECE569_Adjoint(T) * Blist(:, i);
end
end
