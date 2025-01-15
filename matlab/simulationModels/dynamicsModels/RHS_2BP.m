function [dxdt] = RHS_2BP(~, x, i_strDynParams)

dxdt = [x(4:6);
    -i_strDynParams.dGravParam/norm(x(1:3))^3 * x(1:3)];

end
