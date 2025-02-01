function [o_dGradU, o_dPlm] = ExtSHE_GradU(i_dRSCnorm, i_dSCLat, i_dSCLong, i_ui8lMax, i_strParams) %#codegen
%% PROTOTYPE
% [o_dGradU, o_dPlm] = ExtSHE_GradU(i_dRSCnorm, i_dSCLat, i_dSCLong, i_ui8lMax, i_strParams) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function computing the derivative of the disturbing function R for the
% non-spherical terms of the gravitational potential U. The model does NOT
% contain the spherical (central force) contribution, which must be added
% separately. NON-NORMALIZED coefficients are required as input.
% NOTE: (NOT YET IMPLEMENTED) conversion from normalized coefficients is 
% provided as feature if ExtSHE_normFactors() is available, but it is 
% strongly discouraged due to increase in computational time. 
% Set "param.normFlag" to 1 if desired. The latter functionality supports 
% up to order lMax = 84.
% REFERENCE: 
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% i_dRSCnorm:  [1] Position vector magnitude in Target Body fixed frame
% i_dSCLat:    [1] Latitude of SC in Target Body fixed frame
% i_dSCLong:   [1] Longitude of SC in Target Body fixed frame
% i_ui8lMax:   [1] Maximum degree of SHE
% i_strParams: [struct] Data structure containing the data required by the SHE:
%              with fields: 
%                 1) params.mu: Gravitational parameters of the body
%                 2) params.meanRbody: mean radius of the body
%                 3) params.Clm_LUT: Clm coefficients Look-up Table
%                 4) params.Slm_LUT: Slm coefficients Look-up Table
% ACHTUNG: Clm, Slm are assumed to start from C11, S11 as first entries.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% o_dGradU: [3x1] Gradient of Potential Function U not including Spherical
%              (central) force of the body.
% o_dPlm:   [(lMax+2)(lMax+2+1)/2, 1] Array of Legendre Polynomials values
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 17-07-2023    Pietro Califano     Function coded. Run-time verified.
% 18-08-2023    Pietro Califano     Compatibility for code generation.
% 30-11-2023    Pietro Califano     Function validated with EGM2008 against
%                                   MATLAB achieving difference of less
%                                   than 0.05% with more than 10 speed up.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ExtSHE_normFactors()
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 1) Implementation of conversion of coefficients to unnormalized 
%    at run-time.
% 2) Removing struct input for better "equivalency" with C.
% -------------------------------------------------------------------------------------------------------------

%% Unpack inputs
dMu = i_strParams.mu; %[LU^3/TU^2]
dMeanRbody = i_strParams.meanRbody; % [LU]
% Check if conversion is required by the user
% if isfield(params, "normFlag") && params.normFlag == 1
% 
%     % EXPERIMENTAL FEATURE
%     Clm_LUTbar = params.Clm_LUT;
%     Slm_LUTbar = params.Slm_LUT;
% 
%     % Compute conversion factors (WARNING: this increases a lot the computational
%     % time. It is adviced to convert the coefficients prior to input)
%     scalingFactor = ExtSHE_normFactors(lMax);
%     % Scale coefficients
%     Clm_LUT = Clm_LUTbar./scalingFactor;
%     Slm_LUT = Slm_LUTbar./scalingFactor;
%     % Reset nan to 0
%     Clm_LUT(isnan(Clm_LUT)) = 0;
%     Slm_LUT(isnan(Slm_LUT)) = 0;
% else
ClmLUT = i_strParams.Clm_LUT;
SlmLUT = i_strParams.Slm_LUT;
% end

%% Allocate variables
% l: degree
% m: order

% Generate Look-Up Table of trigonometric functions
% trigFcnValuesTables(:) = computeTrigFcnValues(mMax, i_dSCLat, i_dSCLong);
dSHEtrigLUT = buildSHEtrigLUT(i_ui8lMax, i_dSCLat, i_dSCLong);

% Generate Look-Up Table of Legendre Polynomials
% o_dPlmLUT = RecursiveLegendrePoly(sin(i_dSCLat), cos(i_dSCLat), i_ui8lMax);
[o_dPlm, ui16IdPlm0] = RecursiveLegendrePolyColVec(sin(i_dSCLat), cos(i_dSCLat), i_ui8lMax);

% Extract LUT of trigonometric functions
sin_mLongLUT = dSHEtrigLUT(:, 1);
cos_mLongLUT = dSHEtrigLUT(:, 2);
mtan_LatLUT = dSHEtrigLUT(:, 3);
% clear trigFcnValuesTables

% Initialize gradient
dUdr = 0.0;
dUdLat = 0.0;
dUdLong = 0.0;

% Compute Radii ratio
% R_body2R_SC = meanRbody/rSC;
idPair = uint16(1); % Pointer to first coefficients in Clm and Slm LUT
for l = 2:i_ui8lMax

    R_body2R_SC_l = (dMeanRbody/i_dRSCnorm)^l; %R_body2R_SC^l;
    R_body2R_SC_laux = R_body2R_SC_l * (l+1);

    % Rework Plm generation to produce a column vector: it's less prone to
    % errors with the format of Clm and Slm as columns.
    for m = 0:l
        idm = uint16(m) + uint16(1);
        % Retrieve P_lm and P_lmPlus1 from LUT
        % (idl, idm)=(1, 1) corresponds to (l, m)=(0, 0)
        idLm0 = ui16IdPlm0(l+1); % Corresponds to Plm0
        P_lm = o_dPlm(idLm0 + uint16(m));
        P_lmPlus1 = o_dPlm(idLm0 + idm); % Note: Plm_LUT must be computed up to l = lMax + 1 in order for m+1 to be available

        % Get amplitude coefficients C_lm and S_lm
        C_lm = ClmLUT(idPair + uint16(1));
        S_lm = SlmLUT(idPair + uint16(1));
    
        % Sum dUdr harmonics
        dUdr = dUdr + R_body2R_SC_laux * P_lm * ( C_lm * cos_mLongLUT(idm) + S_lm * sin_mLongLUT(idm) );
        % Sum dUdLong harmonics
        dUdLat = dUdLat + R_body2R_SC_l * ( P_lmPlus1 - mtan_LatLUT(idm) * P_lm ) * ...
        ( C_lm * cos_mLongLUT(idm) + S_lm * sin_mLongLUT(idm) );
        % Sum dUdLat harmonics
        dUdLong = dUdLong + R_body2R_SC_l * m * P_lm * ( S_lm * cos_mLongLUT(idm) - C_lm * sin_mLongLUT(idm) );
        
        % Update extraction pointer
        idPair = idPair + uint16(1);
    end

end

% Multiply by external coefficient
dUdr = -dMu/i_dRSCnorm^2 * dUdr;
dUdLat = dMu/i_dRSCnorm * dUdLat; % dUdPhi on Vallado
dUdLong = dMu/i_dRSCnorm * dUdLong; % dUdlambda on Vallado

% Assign output vector
o_dGradU = [dUdr; dUdLat; dUdLong];

%% LOCAL FUNCTIONS
    function [o_dPlm, o_ui16IdPlm0] = RecursiveLegendrePolyColVec(i_dGamma, i_dDerGamma, i_ui8lMax) %#codegen
        %% PROTOTYPE
        % [o_dPlm, o_ui16IdPlm0] = RecursiveLegendrePolyColVec(i_dGamma, i_dDerGamma, i_ui8lMax)
        % -------------------------------------------------------------------------------------------------------------
        %% DESCRIPTION
        % Function computing a column vector containing all the non zero derived
        % Legendre Polynomials up to order lMax+2, for i_dGamma. i_dDerGamma must
        % be the derivative of i_dGamma for the generation to be consistent.
        % Recursive formulas are used. Coding and verification specifically
        % for SHE model; therefore the generation goes up to lMax + 2.
        %
        % Reference:
        % 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
        %
        % NOTE 1: the function is tailored for use with Exterior SHE model by
        % providing gamma and dgamma (derivative of gamma) equal to sin(Lat) and
        % cos(Lat) as inputs. However, it is completely general provided that
        % i_dGamma and i_dDerGamma are consistent.
        % -------------------------------------------------------------------------------------------------------------
        %% INPUT
        % i_dGamma:    [1] Evaluation point of the Polynomial.
        %                  For SHE, gamma = sin(Phi_latitude)
        % i_dDerGamma: [1] Derivative of gamma. For SHE, dgamma = cos(Phi_latitude)
        % i_ui8lMax:   [1] Maximum degree of the Polynomial to compute
        % -------------------------------------------------------------------------------------------------------------
        %% OUTPUT
        % o_dPlm:       [(lMax+2)(lMax+2+1)/2, 1] Column vector of the Derived Legendre
        %                                         Polynomials of gamma. Degree l along
        %                                         row, order m along column.
        % o_ui16IdPlm0: [lMax+1, 1] Pointers to Plm with m = 0
        % -------------------------------------------------------------------------------------------------------------
        %% CHANGELOG
        % 30-11-2023    Pietro Califano    Redesigned from LUT version for speed
        %                                  and memory savings. Verified up to l=3.
        % -------------------------------------------------------------------------------------------------------------
        %% DEPENDENCIES
        % [-]
        % -------------------------------------------------------------------------------------------------------------
        %% Future upgrades
        % 1) For convenience of use, one may add additional code (decreasing
        % efficiency) to provide the pointers to every Plm at m=0.
        % -------------------------------------------------------------------------------------------------------------

        %% Function code
        % Initialize output
        ui16Nentries = uint16((i_ui8lMax+2) * (i_ui8lMax+3)/2);
        o_dPlm = zeros(ui16Nentries, 1, 'double'); % nan ONLY FOR DEBUG
        o_ui16IdPlm0 = zeros(i_ui8lMax+2, 1, 'uint16');

        % Recursion Start-up values
        o_dPlm(1) = 1.0; % l=0, m=0
        o_dPlm(2) = i_dGamma; % l=1, m=0
        o_dPlm(3) = i_dDerGamma; % l=1, m=1

        % Initialize pointers
        idSave = uint16(4);
        idlmin1m0 = uint16(2);
        idlmin2m0 = uint16(1);

        o_ui16IdPlm0(1:2) = uint16([1; 2]);
        % Recursion nested loops over (l,m)
        for lDeg = 2:i_ui8lMax+1
            % Reset m counter
            idlprevmCounter = uint16(0);
            for mOrd = uint16(0:lDeg)
                if mOrd == 0
                    % Compute Legendre Polynomial of Zonal Harmonics (m=0)
                    o_dPlm(idSave) = ((2*lDeg-1) * i_dGamma * o_dPlm(idlmin1m0) - (lDeg-1) * o_dPlm(idlmin2m0))/lDeg;
                    o_ui16IdPlm0(lDeg+1) = idSave;
                elseif mOrd > 0 && mOrd < lDeg
                    if mOrd > lDeg-2
                        dPlmin2m = 0.0;
                    else
                        dPlmin2m = o_dPlm(idlmin2m0 + idlprevmCounter);
                    end
                    % Compute Legendre Polynomail of Sectoral Harmonics (m in [1, l-1])
                    o_dPlm(idSave) = dPlmin2m + (2*lDeg-1) * i_dDerGamma * o_dPlm(idlmin1m0 + idlprevmCounter - uint16(1));
                else
                    % Compute Legendre Polynomial for m=l (diagonal of LUT array)
                    o_dPlm(idSave) = (2*lDeg-1) * i_dDerGamma * o_dPlm(idlmin1m0 + idlprevmCounter - uint16(1));
                end
                % Increase pointer to m
                idlprevmCounter = idlprevmCounter + uint16(1);
                % Increase saving pointer
                idSave = idSave + uint16(1);
            end
            % Update pointers to previous degrees
            idlmin2m0 = idlmin1m0;
            idlmin1m0 = idSave - idlprevmCounter;
        end

    end

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
        % o_dSHEtrigLUT: [lMax, 3] Array of recursively computed sine,
        %                 cosine and mtangent of geocentric Latitude
        % -------------------------------------------------------------------------------------------------------------
        %% CHANGELOG
        % 29-11-2023    Pietro Califano     New version of previous prototype.
        % -------------------------------------------------------------------------------------------------------------
        %% DEPENDENCIES
        % [-]
        % -------------------------------------------------------------------------------------------------------------
        %% Future upgrades
        % [-]
        % -------------------------------------------------------------------------------------------------------------
        %% Function code

        % Initialize array
        o_dSHEtrigLUT = zeros(i_ui8mMax+1, 3); % Must be +2 not +1 (check if idSave exceeds pre-allocation to verify).

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

        idSave = uint16(3);

        % Recursion loop
        for mLoop = 2:i_ui8mMax+1

            % Sin recursion
            o_dSHEtrigLUT(idSave, 1) = 2*cosLong * o_dSHEtrigLUT(idSave-uint16(1), 1) - ...
                o_dSHEtrigLUT(idSave-2, 1);

            % Cosine recursion
            o_dSHEtrigLUT(idSave, 2) = 2*cosLong * o_dSHEtrigLUT(idSave-uint16(1), 2) - ...
                o_dSHEtrigLUT(idSave-2, 2);

            % mTangent recursion
            o_dSHEtrigLUT(idSave, 3) = o_dSHEtrigLUT(idSave-uint16(1), 3) + tanLat;

            % Increase counter by 1
            idSave = idSave + uint16(1);
        end

        % Check for numerical zeros and insert true zeros
        o_dSHEtrigLUT( (abs(o_dSHEtrigLUT) - eps) < 1.5 * eps ) = 0;

    end

end