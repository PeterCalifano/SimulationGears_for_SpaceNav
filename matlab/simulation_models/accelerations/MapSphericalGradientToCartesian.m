function dGradUcart = MapSphericalGradientToCartesian(dPosSC_TB, dGradUsph) %#codegen
%% PROTOTYPE
% dGradUcart = MapSphericalGradientToCartesian(dPosSC_TB, dGradUsph) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Maps spherical-coordinate potential gradients at a target-frame position
% to Cartesian gradients in the same frame.
%
% dGradUsph may contain one or more gradient columns with rows
% [dU/dr; dU/dLat; dU/dLong]. The returned dGradUcart has the same number
% of columns.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dPosSC_TB:     [3 x 1]   Position in the target-body fixed frame.
% dGradUsph:     [3 x N]   Spherical-coordinate potential-gradient columns.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dGradUcart:    [3 x N]   Cartesian potential-gradient columns.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 26-04-2026    Pietro Califano     Extract shared spherical-to-Cartesian gradient mapping.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
% Input checks
dPosSCnorm = norm(dPosSC_TB);
if dPosSCnorm <= 0.0
    error('MapSphericalGradientToCartesian:ZeroPosition', ...
        'Position vector must be non-zero.');
end

ui32NumCols = uint32(size(dGradUsph, 2));
dGradUcart = zeros(3, double(ui32NumCols));

dUdr = dGradUsph(1, :);
dUdLat = dGradUsph(2, :);
dUdLong = dGradUsph(3, :);

% Handle the polar singularity explicitly when the in-plane radius tends to zero
dRho2 = dPosSC_TB(1)^2 + dPosSC_TB(2)^2;
if dRho2 <= (16.0 * eps(dPosSCnorm) * dPosSCnorm)^2
    dGradUcart(3, :) = dUdr * dPosSC_TB(3) / dPosSCnorm;
    return;
end

% Compute the Cartesian gradient using the chain rule and the spherical-to-Cartesian Jacobian
dRho = sqrt(dRho2);
dInvR = 1.0 / dPosSCnorm;
dInvR2 = dInvR * dInvR;
dCommon = dUdr * dInvR - dPosSC_TB(3) * dUdLat / (dPosSCnorm^2 * dRho);

dGradUcart(1, :) = dCommon * dPosSC_TB(1) - dUdLong * dPosSC_TB(2) / dRho2;
dGradUcart(2, :) = dCommon * dPosSC_TB(2) + dUdLong * dPosSC_TB(1) / dRho2;
dGradUcart(3, :) = dUdr * dPosSC_TB(3) * dInvR + dUdLat * dRho * dInvR2;

end
