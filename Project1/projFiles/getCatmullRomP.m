% Catmull-Rom Interpolation
% Given u, P_i0, P_i1, P_i2, P_i3
% Positions can be a vector of values
function Pu=getCatmullRomP(u, p_i0, p_i1, p_i2, p_i3)
    U = [u^3,u^2,u,1];
    H = [-.5,1.5,-1.5,.5; 1,-2.5,2,-.5; -.5,0,.5,0; 0,1,0,0];
    P = [p_i0; p_i1; p_i2; p_i3];
    Pu = U*H*P;
end