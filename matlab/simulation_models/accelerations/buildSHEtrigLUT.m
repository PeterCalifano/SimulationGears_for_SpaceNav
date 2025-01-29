function o_dSHEtrigLUT = buildSHEtrigLUT(i_ui8mMax, i_dLat, i_dLong) %#codegen
%% PROTOTYPE
% o_dSHEtrigLUT = buildSHEtrigLUT(i_ui8mMax, i_dLat, i_dLong) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function to build trigonometric functions "Look-Up table" from recursion
% equation, for the inner loop of the Exterior SHE model
% (order from 0 to lMax).
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% i_ui8mMax: [1] Maximum order of SHE model (equal to lMax)
% i_dLat:    [1] Latitude of SC in Target Body fixed frame
% i_dLong:   [1] Longitude of SC in Target Body fixed frame
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% o_dSHEtrigLUT: [lMax, 3]
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 29-11-2023    Pietro Califano     New version of previous protot\ype.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

% Initialize array
o_dSHEtrigLUT = zeros(i_ui8mMax+1, 3);

% One-time computations of variables
cosLong = cos(i_dLong); % Longitude Lambda
tanLat = tan(i_dLat); % Geocentric Latitude phi

% Pre-loop computations
% Row 1 entries
o_dSHEtrigLUT(1, 1) = 0.0;
o_dSHEtrigLUT(1, 2) = 1.0;
o_dSHEtrigLUT(1, 3) = 0.0;

% Row 2 entries
o_dSHEtrigLUT(2, 1) = sin(i_dLong);
o_dSHEtrigLUT(2, 2) = cosLong;
o_dSHEtrigLUT(2, 3) = tanLat;

idSave = 3;

% Recursion loop
for m = 3:i_ui8mMax

    % Sin recursion
    o_dSHEtrigLUT(idSave, 1) = 2*cosLong * o_dSHEtrigLUT(idSave-1, 1) - ...
        o_dSHEtrigLUT(idSave-2, 1);

    % Cosine recursion
    o_dSHEtrigLUT(idSave, 2) = 2*cosLong * o_dSHEtrigLUT(idSave-1, 2) - ...
        o_dSHEtrigLUT(idSave-2, 2);

    % mTangent recursion
    o_dSHEtrigLUT(idSave, 3) = o_dSHEtrigLUT(idSave-1, 3) + tanLat;

    % Increase counter by 1
    idSave = idSave + 1;
end

% Check for numerical zeros and insert true zeros
o_dSHEtrigLUT( (abs(o_dSHEtrigLUT) - eps) < 1.5 * eps ) = 0;


end





