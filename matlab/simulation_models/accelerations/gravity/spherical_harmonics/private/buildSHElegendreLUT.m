function [dPlm, ui32IdPlm0] = buildSHElegendreLUT(dGamma, dDerGamma, ui32lMax) %#codegen
%% PROTOTYPE
% [dPlm, ui32IdPlm0] = buildSHElegendreLUT(dGamma, dDerGamma, ui32lMax) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function computing a column-vector lookup table of the non-zero Legendre
% polynomial terms required by the Exterior SHE model.
%
% The output stores the terms degree-by-degree in a flattened triangular
% layout and returns the pointer to the m = 0 entry for each degree.
%
% The function is tailored for ExtSHE by using:
%   dGamma    = sin(latitude)
%   dDerGamma = cos(latitude)
%
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% INPUTS
% dGamma:        [1] Evaluation point of the polynomial recursion.
% dDerGamma:     [1] Derivative of dGamma.
% ui32lMax:      [1] Maximum harmonic degree.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUTS
% dPlm:          [N x1]   Flattened lookup table of Legendre polynomial values.
% ui32IdPlm0:    [lMax+2] Pointers to the m = 0 entry for each degree.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Function code
ui32lMax = uint32(ui32lMax);

ui32Nentries = (ui32lMax + uint32(2)) * (ui32lMax + uint32(3)) / uint32(2);
dPlm = zeros(ui32Nentries, 1, 'double');
ui32IdPlm0 = zeros(ui32lMax + uint32(2), 1, 'uint32');

% Recursion seeds
dPlm(1) = 1.0;         % l = 0, m = 0
dPlm(2) = dGamma;      % l = 1, m = 0
dPlm(3) = dDerGamma;   % l = 1, m = 1

ui32IdPlm0(1:2) = uint32([1; 2]);

idSave = uint32(4);
idlMin1m0 = uint32(2);
idlMin2m0 = uint32(1);

for lDeg = 2:(double(ui32lMax) + 1.0)
    idPrevDegree = uint32(0);

    for mOrd = 0:lDeg
        
        if mOrd == 0
            dPlm(idSave) = ((2.0 * lDeg - 1.0) * dGamma * dPlm(idlMin1m0) ...
                - (lDeg - 1.0) * dPlm(idlMin2m0)) / lDeg;
            ui32IdPlm0(uint32(lDeg) + uint32(1)) = idSave;

        elseif mOrd < lDeg

            if mOrd > (lDeg - 2.0)
                dPlmLMinus2 = 0.0;
            else
                dPlmLMinus2 = dPlm(idlMin2m0 + idPrevDegree);
            end

            dPlm(idSave) = dPlmLMinus2 + (2.0 * lDeg - 1.0) * dDerGamma ...
                * dPlm(idlMin1m0 + idPrevDegree - uint32(1));
        
            else
            dPlm(idSave) = (2.0 * lDeg - 1.0) * dDerGamma ...
                * dPlm(idlMin1m0 + idPrevDegree - uint32(1));
        end

        idPrevDegree = idPrevDegree + uint32(1);
        idSave = idSave + uint32(1);
    end

    idlMin2m0 = idlMin1m0;
    idlMin1m0 = idSave - idPrevDegree;
end
end
