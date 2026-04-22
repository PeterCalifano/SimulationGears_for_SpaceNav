function [dPlm, ui32IdPlm0] = buildSHElegendreLUT(dGamma, dDerGamma, ui32lMax) %#codegen
%% DESCRIPTION
% buildSHElegendreLUT Build unnormalized associated Legendre values for ExtSHE.
% The output stores the non-zero P_lm terms degree-by-degree in a column
% vector and returns the offset of the m=0 term for each degree.

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
