function dJacSHE_TB = EvalJac_ExtSphericalHarmExp(dRSC_TB, ...
    ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
%% PROTOTYPE
% dJacSHE_TB = EvalJac_ExtSphericalHarmExp(dRSC_TB, ui32MaxDegree, ...
%     dCSlmCoeffCols, dGravParam, dBodyRadiusRef) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Compatibility wrapper for the canonical target-frame spherical-harmonics Jacobian evaluator.
% -------------------------------------------------------------------------------------------------------------

dJacSHE_TB = EvalJac_ExtSphHarmExpInTargetFrame(dRSC_TB, ...
    ui32MaxDegree, dCSlmCoeffCols, dGravParam, dBodyRadiusRef);

end
