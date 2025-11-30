function Tinv = ECE569_TransInv(T)
% Inverse of homogeneous transform T = [R p; 0 1]
% T^{-1} = [R^T, -R^T p; 0 1]
R = T(1:3,1:3);
p = T(1:3,4);
Tinv = [R.', -R.'*p; 0 0 0 1];
end
