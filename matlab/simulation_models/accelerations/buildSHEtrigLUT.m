function dSHEtrigLUT = buildSHEtrigLUT(ui32mMax, dLat, dLong) %#codegen
%% PROTOTYPE
% dSHEtrigLUT = buildSHEtrigLUT(ui32mMax, dLat, dLong) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function to build the trigonometric lookup table for the inner loop of
% the Exterior SHE model.
%
% The table is generated recursively over the harmonic order m and stores:
%   column 1: sin(m * longitude)
%   column 2: cos(m * longitude)
%   column 3: m * tan(latitude)
%
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% INPUTS
% ui32mMax:      [1] Maximum harmonic order (equal to lMax in ExtSHE).
% dLat:          [1] Geocentric latitude [rad].
% dLong:         [1] Longitude [rad].
% -------------------------------------------------------------------------------------------------------------
%% OUTPUTS
% dSHEtrigLUT:   [(ui32mMax+2) x 3] Lookup table. Row k corresponds to
%                harmonic order m = k-1.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

ui32mMax = uint32(ui32mMax);

dSHEtrigLUT = zeros(ui32mMax + uint32(2), 3);

dCosLong = cos(dLong);
dTanLat = tan(dLat);

dSHEtrigLUT(1, 1) = 0.0;
dSHEtrigLUT(1, 2) = 1.0;
dSHEtrigLUT(1, 3) = 0.0;

dSHEtrigLUT(2, 1) = sin(dLong);
dSHEtrigLUT(2, 2) = dCosLong;
dSHEtrigLUT(2, 3) = dTanLat;

for idxRow = 3:(double(ui32mMax) + 2.0)
    dSHEtrigLUT(idxRow, 1) = 2.0 * dCosLong * dSHEtrigLUT(idxRow - 1, 1) ...
                            - dSHEtrigLUT(idxRow - 2, 1);
    dSHEtrigLUT(idxRow, 2) = 2.0 * dCosLong * dSHEtrigLUT(idxRow - 1, 2) ...
                            - dSHEtrigLUT(idxRow - 2, 2);
    dSHEtrigLUT(idxRow, 3) = dSHEtrigLUT(idxRow - 1, 3) + dTanLat;
end

dSHEtrigLUT(abs(dSHEtrigLUT) <= 0.1 * eps('double')) = 0.0;
end
