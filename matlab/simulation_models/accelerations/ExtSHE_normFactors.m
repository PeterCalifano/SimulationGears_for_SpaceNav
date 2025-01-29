function scaleFactors = ExtSHE_normFactors(lMax) %#codegen
arguments
    lMax (1,1) double
end
%% PROTOTYPE
% [scaleFactors] = ExtSHE_normFactors(lMax)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function computing the scaling factors of the SHE model coefficients to convert between Normalized
% and Non-normalized (scaled) coefficients, typically indicated as (Clm-bar, Slm-bar) and (Clm, Slm)
% respectively. A 1D array containing the scaling coefficients for each entry of the Clm and Slm as
% required by function ExtSHE_GradU(), is produced.
% The function returns the scaling coefficients from (1,1) to (lMax, lMax) (degree, order).
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 18-08-2023    Pietro Califano     Function coded. Run-time verified. Output validated with Vallado
%                                   Appendix D4 tables.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% lMax:         [1]                 Maximum required degree of the expansion
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% scaleFactors: [sum((2:lMax)+1)+1]   Number of scaleFactors, function of lMax
% -------------------------------------------------------------------------------------------------------------

%% Function code
% Compute number of coefficients
MAX_ALLOWABLE_DEGREE = 83;
arraySize = sum((2:lMax)+1)+1;

if lMax <= MAX_ALLOWABLE_DEGREE

    % Initialize storage arrays
    scaleFactors = zeros(arraySize, 1);
    scaleFactors(1) = 1; % Coefficient for l=1, m=1

    % Compute Scaling factors for each (l, m) pairs: (2, 0) to (lMax, lMax)
    idPair = 2;
    for l = 2:lMax % NOTE: l and m are the actual degree and order, not the indices.
        for m = 0:l
            if m > 0
                scaleFactors(idPair) = sqrt(factorial(l+m) /...
                    ( (factorial(l-m)) * 2 * (2*l+1) ));

            else
                scaleFactors(idPair) = sqrt(factorial(l+m) /...
                    ( (factorial(l-m)) * 1 * (2*l+1) ));
            end
            idPair = idPair + 1;
        end
    end
    if (idPair-1 ~= arraySize)
        error('Mismatch between precomputed arraySize and allocator index idPair. Result may be incorrect.')
    end
else
    error('Current implementation does not support order > 84 due to factorial() limitation.');
end

end

