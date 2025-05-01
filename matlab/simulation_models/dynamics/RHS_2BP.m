function [dxdt] = RHS_2BP(~, x, strDynParams)

dxdt = [x(4:6);
    -strDynParams.dGravParam/norm(x(1:3))^3 * x(1:3)];

end
