function dValues = ComputeLogSpacedValues(dStartValue, dEndValue, ui32NumValues)%#codegen
arguments
    dStartValue     (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dEndValue       (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    ui32NumValues   (1,1) uint32
end
%% PROTOTYPE
% dValues = ComputeLogSpacedValues(dStartValue, dEndValue, ui32NumValues)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Generates a row vector of logarithmically spaced values between the input
% bounds. Consecutive values have a constant ratio.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dStartValue:    [1,1]         Initial value.
% dEndValue:      [1,1]         Final value.
% ui32NumValues:  [1,1]         Number of requested samples.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dValues:        [1 x N]     Logarithmically spaced values from
%                             dStartValue to dEndValue.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 25-04-2026    Pietro Califano     Rename helper to clarify log-spaced behaviour.
% 24-04-2026    Pietro Califano     Promote helper from SH fitter to shared utility.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

if ui32NumValues == uint32(0)
    error('ComputeLogSpacedValues:InvalidCount', ...
        'ui32NumValues must be at least 1.');
end

if ui32NumValues == uint32(1)
    dValues = dStartValue;
    return;
end

dRatio = (dEndValue / dStartValue)^(1.0 / double(ui32NumValues - uint32(1)));
dValues = zeros(1, double(ui32NumValues));
dValues(1) = dStartValue;

for idValue = 2:double(ui32NumValues)
    dValues(idValue) = dValues(idValue - 1) * dRatio;
end

end
