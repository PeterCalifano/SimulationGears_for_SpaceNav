function [dPotentialPert, dAccPertTB] = EvalExtSphHarmSamplesInTargetFrame( ...
    dSamplePos_TB, ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
arguments
    dSamplePos_TB       (3,:) double {mustBeFinite, mustBeReal}
    ui32MaxDegree       (1,1) uint32
    dCSlmCoeffCols      (:,2) double {mustBeFinite, mustBeReal}
    dGravParam          (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
    dBodyRadiusRef      (1,1) double {mustBeFinite, mustBeReal, mustBePositive}
end
%% PROTOTYPE
% [dPotentialPert, dAccPertTB] = EvalExtSphHarmSamplesInTargetFrame( ...
%     dSamplePos_TB, ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluates the canonical exterior spherical harmonics perturbation model
% on a set of target-frame sample positions.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dSamplePos_TB:       [3 x N]         Sample positions in target-body frame.
% ui32MaxDegree:       [1]             Maximum harmonic degree.
% dCSlmCoeffCols:      [Nl x 2]        Unnormalized [Clm, Slm] coefficient table.
% dGravParam:          [1]             Gravitational parameter.
% dBodyRadiusRef:      [1]             Reference radius.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPotentialPert:      [N x 1]         Perturbative potential samples.
% dAccPertTB:          [3 x N]         Perturbative acceleration samples.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 24-04-2026    Pietro Califano     Add shared multi-sample canonical SHE evaluator.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% EvalExtSphHarmExpInTargetFrame()
% -------------------------------------------------------------------------------------------------------------

ui32NumSamples = uint32(size(dSamplePos_TB, 2));
dPotentialPert = zeros(double(ui32NumSamples), 1);
dAccPertTB = zeros(3, double(ui32NumSamples));

% Evaluate the potential and acceleration at each sample position
for idSample = 1:double(ui32NumSamples)

    [dPotentialPert(idSample), dAccPertTB(:, idSample)] = EvalExtSphHarmExpInTargetFrame( ...
        dSamplePos_TB(:, idSample), ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef);

end

end
