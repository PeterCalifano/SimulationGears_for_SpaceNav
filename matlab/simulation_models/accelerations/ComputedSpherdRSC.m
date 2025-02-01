function dSphdRSC = ComputedSpherdRSC(R_SC, rSC)%#codegen
% Function computing the derivatives of the Spherical coordinate (r,
% Long, Lat) with respect to the position vector R_SC, for the Chain
% Rule required for the computation of the acceleration

% R_SC must be a column vector
% Function verification PASSED

% Derivative of the range wrt Position vector
drdRSC = R_SC./rSC;
% Derivative of Longitude Phi wrt Position vector
invsqrtIJ = 1/sqrt(R_SC(1)^2 + R_SC(2)^2);

dLatdRSC = invsqrtIJ * (-R_SC * R_SC(3)./rSC^2 +  [0; 0; 1]);
% Derivative of Latitude Lambda wrt Position vector
dLongdRSC = 1/(R_SC(1)^2 + R_SC(2)^2) * (R_SC(1) * [0; 1; 0] - R_SC(2) * [1; 0; 0]);

% Stack output (horizontally, column vectors)
dSphdRSC = [drdRSC, dLatdRSC, dLongdRSC];
end