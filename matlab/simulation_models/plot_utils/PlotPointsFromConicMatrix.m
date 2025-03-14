function [o_dlimbPixCoords, o_strConicData] = PlotPointsFromConicMatrix(dtightConeLocusImageMatrix, ...
    dRangeOfAngles, ui16Npoints)
%% PROTOTYPE
% [o_dlimbPixCoords, o_strConicData] = PlotPointsFromConicMatrix(dtightConeLocusImageMatrix, ...
%     dRangeOfAngles, ui16Npoints)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dtightConeLocusImageMatrix
% dRangeOfAngles
% ui16Npoints
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% o_dlimbPixCoords
% o_strConicData
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 16-07-2024        Pietro Califano         Moved code to function
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

% Extract coefficients from the matrix (from GPT 4o)
A = dtightConeLocusImageMatrix(1,1); % A must be > 0
B = 2 * dtightConeLocusImageMatrix(1,2);
C = dtightConeLocusImageMatrix(2,2); % C must be > 0
D = 2 * dtightConeLocusImageMatrix(1,3);
E = 2 * dtightConeLocusImageMatrix(2,3);
F = dtightConeLocusImageMatrix(3,3);

assert(A>=0, "A coefficient must be >= 0")
assert(C>=0, "C coefficient must be >= 0")

% Calculate the center of the ellipse
Delta = B^2 - 4*A*C;
ellipseCx = (2*C*D - B*E) / Delta; % Should be around image centre if Moon is near boresight
ellipseCy = (2*A*E - B*D) / Delta;

% Calculate the angle of rotation
majorAxisAngleFromX = 0.5 * double(atan2(-real(B), real(C-A)));

% Calculate the semi-major and semi-minor axes
aTmp = -sqrt(2*(A*E^2+C*D^2-B*D*E+F*(Delta))*(A+C+sqrt((A-C)^2+B^2)))/Delta;
bTmp = -sqrt(2*(A*E^2+C*D^2-B*D*E+F*(Delta))*(A+C-sqrt((A-C)^2+B^2)))/Delta;

if aTmp > bTmp
    semiMajorAx = aTmp;
    semiMinorAx = bTmp;

else
    semiMajorAx = bTmp;
    semiMinorAx = aTmp;

    majorAxisAngleFromX = pi/2 + majorAxisAngleFromX;
end

o_strConicData.dEllipseCentre       = [ellipseCx; ellipseCy];
o_strConicData.dSemiMajorAx         = semiMajorAx;
o_strConicData.dSemiMinorAx         = semiMinorAx;
o_strConicData.dMajorAxisAngleFromX = majorAxisAngleFromX;

% Parametric equation of the ellipse
if ui16Npoints == 0 && length(dRangeOfAngles) > 2
    pixAnglesToGet = dRangeOfAngles;

elseif ui16Npoints > 0 && length(dRangeOfAngles) == 2
    pixAnglesToGet = linspace(dRangeOfAngles(1), dRangeOfAngles(2), ui16Npoints);

else
    fprintf("\n Warning: incorrect inputs to get pixels! Using default value: [0, 2pi], 1000 points.")
    pixAnglesToGet = linspace(0, 2*pi, 1000);
end

% Compute pixels
o_dlimbPixCoords = coder.nullcopy(zeros(2, length(pixAnglesToGet)));

% sinMajorAxFromX = sin(majorAxisAngleFromX);
% cosMajorAxFromX = cos(majorAxisAngleFromX);

% Rotation matrix
% R = [cosMajorAxFromX, -sinMajorAxFromX;
%     sinMajorAxFromX, cosMajorAxFromX];

R = eye(2);

xEllipse = semiMajorAx * cos(pixAnglesToGet);
yEllipse = semiMinorAx * sin(pixAnglesToGet);

% Rotate the points
rotatedPoints = R * [xEllipse; yEllipse];
o_dlimbPixCoords(1, :) = ellipseCx + rotatedPoints(1, :);
o_dlimbPixCoords(2, :) = ellipseCy + rotatedPoints(2, :);

end
