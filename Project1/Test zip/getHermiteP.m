% Hermite Interpolation
% Given u, P_i, P_i+1, D_i, D_i+1
% Position and velocities can be a vector of values
function Q=getHermiteP(u,p_i,p_i1,d_i,d_i1)
    U = [u^3, u^2, u^1, 1];
    H = [2, -2, 1, 1;-3 , 3, -2, -1;0, 0, 1, 0;1, 0, 0, 0];
    jointInfo = [p_i; p_i1; d_i; d_i1];
    Q = U*H*jointInfo;
end