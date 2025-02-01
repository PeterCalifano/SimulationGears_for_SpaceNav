function o_dSHEaccTB = ExtSHE_AccTB(i_dRSC_TB, ...
    i_ui32MaxDegree, ...
    i_dCSlmCoeffCols, ...
    i_dMu, ...
    i_dBodyRref) %#codegen
arguments
    i_dRSC_TB        (3,1) double {isvector}
    i_ui32MaxDegree  (1,1) uint32 {isscalar}
    i_dCSlmCoeffCols (:,:) double {ismatrix}
    i_dMu            (1,1) double {isscalar}
    i_dBodyRref      (1,1) double {isscalar}
end
%% PROTOTYPE
% o_dSHEaccTB = ExtSHE_AccTB(i_dRSC_TB, i_ui8MaxDegree, i_dCSlmCoeffCols,
%                            i_dMu, i_dBodyRref) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Function implementing the Spherical Harmonics Expansion model to compute
% the perturbative acceleration caused by the Non-Spherical gravitational
% contributions of a generic target body for which the SHE coefficients are
% known (required input). The position vector of the SC in the Target Body
% (TB) FIXED frame must be known. The function only works in the latter
% frame. The output acceleration is expressed in fixed TB as well.

% ACHTUNG: Clm and Slm coefficients must be UN-NORMALIZED. They are 
% assumed to start from C11, S11 as FIRST entries of the column arrays.
% REFERENCE:
% 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% i_dRSC_TB:        [3x1]   Position vector of the SC with respect to the Target body
%                           in the Target body FIXED frame.
% i_ui8MaxDegree:   [1]     Maximum order of Harmonics to consider. Be sure the input
%                           coefficients data is sufficient.
% i_dCSlmCoeffCols: [Nl, 2] (Clm, Slm) coefficients Look-up Table where Nl <= i_ui8MaxDegree.
% i_dMu:            [1]     Gravitational parameters of the body [LU^3/TU^2]
% i_dBodyRref:      [1]     Mean reference radius of the body [LU]
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% o_dSHEaccTB: [3x1] Acceleration vector in the Target Body FIXED frame due
%                    to Spherical harmonics WITHOUT central force.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 17-07-2023    Pietro Califano    Function coded. Run-time verified.
% 30-11-2023    Pietro Califano    Function validated with EGM2008 against MATLAB achieving difference of less
%                                  than 0.05% with more than 10x speed up.
% 15-12-2023    Pietro Califano    Major bug fix for correct code generation and datatypes.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% No external function.
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------

%% Unpack inputs
rSC = sqrt(i_dRSC_TB(1)^2 + i_dRSC_TB(2)^2 + i_dRSC_TB(3)^2);
RSC_unitVec = i_dRSC_TB./rSC;

% Compute SCLat and SCLong in Target body fixed frame given position vector
SCLat = asin(RSC_unitVec(3)); 
SCLong = atan2(RSC_unitVec(2), RSC_unitVec(1));

% Compute Gradient of Gravitational potential wrt Spherical Coords.
GradU = ExtSHE_GradU(rSC, SCLat, SCLong, i_ui32MaxDegree,...
    i_dCSlmCoeffCols, i_dMu, i_dBodyRref);

% Compute derivative of Spherical Coords wrt Position vector
dSphdRSC = ComputedSpherdRSC(i_dRSC_TB, rSC);

% Assign derivative values
% drdRSC = dSphdRSC(:, 1);
% dLatdRSC = dSphdRSC(:, 2);
% dLongdRSC = dSphdRSC(:, 3);

% Compute acceleration terms [3x1] vector in TB frame
o_dSHEaccTB = GradU(1)*dSphdRSC(:, 1) + GradU(2)*dSphdRSC(:, 2) + GradU(3)*dSphdRSC(:, 3);

%% LOCAL FUNCTION
    function dSphdRSC = ComputedSpherdRSC(R_SC, rSC)%#codegen
        % Function computing the derivatives of the Spherical coordinate (r,
        % Long, Lat) with respect to the position vector R_SC, for the Chain
        % Rule required for the computation of the acceleration
        % R_SC must be a column vector

        % Derivative of the range wrt Position vector
        drdRSC = R_SC./rSC;

        % Derivative of Longitude Phi wrt Position vector
        dLatdRSC = 1/sqrt(R_SC(1)^2 + R_SC(2)^2) * (-R_SC * R_SC(3)./rSC^2 +  [0; 0; 1]);

        % Derivative of Latitude Lambda wrt Position vector
        dLongdRSC = 1/(R_SC(1)^2 + R_SC(2)^2) * (R_SC(1) * [0; 1; 0] - R_SC(2) * [1; 0; 0]);

        % Stack output (horizontally, column vectors)
        dSphdRSC = [drdRSC, dLatdRSC, dLongdRSC];
    end

    function [o_dGradU, o_dPlm] = ExtSHE_GradU(i_dRSCnorm, i_dSCLat, i_dSCLong, i_ui32lMax, i_dCSlmCoeffCols, i_dMu, i_dBodyRref) %#codegen
        %% PROTOTYPE
        % [o_dGradU, o_dPlm] = ExtSHE_GradU(i_dRSCnorm, i_dSCLat, i_dSCLong, i_ui32lMax, i_dCSlmCoeffCols, i_dMu, i_dBodyRref) %#codegen
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
        % i_dCSlmCoeffCols: [Nl, 2] (Clm, Slm) coefficients Look-up Table
        %                           where Nl <= i_ui8MaxDegree.
        % i_dMu:            [1] Gravitational parameters of the body [LU^3/TU^2]
        % i_dBodyRref:      [1] Mean reference radius of the body [LU]
        % ACHTUNG: Clm, Slm are assumed to start from C11, S11 as first 
        % entries of the column arrays.
        % -------------------------------------------------------------------------------------------------------------
        %% OUTPUT
        % o_dGradU: [3x1] Gradient of Potential Function U not including Spherical
        %              (central) force of the body.
        % o_dPlm:   [(lMax+2)(lMax+2+3)/2, 1] Array of Legendre Polynomials values
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
%         i_dMu = i_strParams.mu; %[LU^3/TU^2]
%         i_dBodyRref = i_strParams.meanRbody; % [LU]


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
        ClmLUT = i_dCSlmCoeffCols(:, 1);
        SlmLUT = i_dCSlmCoeffCols(:, 2);
        % end

        %% Allocate variables
        % l: degree
        % m: order
        
        % Generate Look-Up Table of trigonometric functions
        % trigFcnValuesTables(:) = computeTrigFcnValues(mMax, i_dSCLat, i_dSCLong);
        dSHEtrigLUT = buildSHEtrigLUT(i_ui32lMax, i_dSCLat, i_dSCLong);

        % Generate Look-Up Table of Legendre Polynomials
        % o_dPlmLUT = RecursiveLegendrePoly(sin(i_dSCLat), cos(i_dSCLat), i_ui8lMax);
        [o_dPlm, ui32IdPlm0] = RecursiveLegendrePolyColVec(sin(i_dSCLat), cos(i_dSCLat), i_ui32lMax);

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
        idPair = uint32(1); % Pointer to first coefficients in Clm and Slm LUT
        for l = 2:i_ui32lMax

            R_body2R_SC_l = (i_dBodyRref/i_dRSCnorm)^double(l); %R_body2R_SC^l;
            R_body2R_SC_laux = R_body2R_SC_l * double(l+1);

            % Rework Plm generation to produce a column vector: it's less prone to
            % errors with the format of Clm and Slm as columns.
            for m = 0:l
                idm = uint32(m) + uint32(1);
                % Retrieve P_lm and P_lmPlus1 from LUT
                % (idl, idm)=(1, 1) corresponds to (l, m)=(0, 0)
                idLm0 = ui32IdPlm0(l+1); % Corresponds to Plm0
                P_lm = o_dPlm(idLm0 + uint32(m));
                P_lmPlus1 = o_dPlm(idLm0 + idm); % Note: Plm_LUT must be computed up to l = lMax + 1 in order for m+1 to be available

                % Get amplitude coefficients C_lm and S_lm
                C_lm = ClmLUT(idPair + uint32(1));
                S_lm = SlmLUT(idPair + uint32(1));

                % Sum dUdr harmonics
                dUdr = dUdr + R_body2R_SC_laux * P_lm * ( C_lm * cos_mLongLUT(idm) + S_lm * sin_mLongLUT(idm) );
                % Sum dUdLong harmonics
                dUdLat = dUdLat + R_body2R_SC_l * ( P_lmPlus1 - mtan_LatLUT(idm) * P_lm ) * ...
                    ( C_lm * cos_mLongLUT(idm) + S_lm * sin_mLongLUT(idm) );
                % Sum dUdLat harmonics
                dUdLong = dUdLong + R_body2R_SC_l * double(m) * P_lm * ( S_lm * cos_mLongLUT(idm) - C_lm * sin_mLongLUT(idm) );

                % Update extraction pointer
                idPair = idPair + uint32(1);
            end

        end

        % Multiply by external coefficient
        dUdr = -i_dMu/i_dRSCnorm^2 * dUdr;
        dUdLat = i_dMu/i_dRSCnorm * dUdLat; % dUdPhi on Vallado
        dUdLong = i_dMu/i_dRSCnorm * dUdLong; % dUdlambda on Vallado

        % Assign output vector
        o_dGradU = [dUdr; dUdLat; dUdLong];

        %% LOCAL FUNCTIONS
        function [o_dPlm, o_ui32IdPlm0] = RecursiveLegendrePolyColVec(i_dGamma, i_dDerGamma, i_ui32lMax) %#codegen
            %% PROTOTYPE
            % [o_dPlm, o_ui16IdPlm0] = RecursiveLegendrePolyColVec(i_dGamma, i_dDerGamma, i_ui32lMax)
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
            % o_dPlm:       [(lMax+2)(lMax+2+3)/2, 1] Column vector of the Derived Legendre
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
            ui32Nentries = uint32((uint32(i_ui32lMax) + 2) * (uint32(i_ui32lMax) + 3)/2);
            o_dPlm = zeros(ui32Nentries, 1, 'double'); % nan ONLY FOR DEBUG
            o_ui32IdPlm0 = zeros(i_ui32lMax + uint32(2), 1, 'uint32');

            % Recursion Start-up values
            o_dPlm(1) = 1.0; % l=0, m=0
            o_dPlm(2) = i_dGamma; % l=1, m=0
            o_dPlm(3) = i_dDerGamma; % l=1, m=1

            % Initialize pointers
            idSave = uint32(4);
            idlmin1m0 = uint32(2);
            idlmin2m0 = uint32(1);

            o_ui32IdPlm0(1:2) = uint32([1; 2]);
            % Recursion nested loops over (l,m)
            for lDeg = 2:double(i_ui32lMax)+1
                % Reset m counter
                idlprevmCounter = uint32(0);
                for mOrd = 0:lDeg
                    if mOrd == 0
                        % Compute Legendre Polynomial of Zonal Harmonics (m=0)
                        o_dPlm(idSave) = ((2*lDeg-1) * i_dGamma * o_dPlm(idlmin1m0) - (lDeg-1) * o_dPlm(idlmin2m0))/double(lDeg);
                        o_ui32IdPlm0(lDeg+1) = idSave;
                    elseif mOrd > 0 && mOrd < lDeg
                        if mOrd > lDeg-2
                            dPlmin2m = 0.0;
                        else
                            dPlmin2m = o_dPlm(idlmin2m0 + idlprevmCounter);
                        end
                        % Compute Legendre Polynomail of Sectoral Harmonics (m in [1, l-1])
                        o_dPlm(idSave) = dPlmin2m + (2*lDeg-1) * i_dDerGamma * o_dPlm(idlmin1m0 + idlprevmCounter - uint32(1));
                    else
                        % Compute Legendre Polynomial for m=l (diagonal of LUT array)
                        o_dPlm(idSave) = (2*lDeg-1) * i_dDerGamma * o_dPlm(idlmin1m0 + idlprevmCounter - uint32(1));
                    end
                    % Increase pointer to m
                    idlprevmCounter = idlprevmCounter + uint32(1);
                    % Increase saving pointer
                    idSave = idSave + uint32(1);
                end
                % Update pointers to previous degrees
                idlmin2m0 = idlmin1m0;
                idlmin1m0 = idSave - idlprevmCounter;
            end

        end

        function o_dSHEtrigLUT = buildSHEtrigLUT(i_ui32mMax, i_dLat, i_dLong) %#codegen
            %% PROTOTYPE
            % o_dSHEtrigLUT = buildSHEtrigLUT(i_ui32mMax, i_dLat, i_dLong) %#codegen
            % -------------------------------------------------------------------------------------------------------------
            %% DESCRIPTION
            % Function to build trigonometric functions "Look-Up table" from recursion
            % equation, for the inner loop of the Exterior SHE model
            % (order from 0 to lMax).
            % REFERENCE:
            % 1) Fundamentals Of Astrodynamics And Applications, Vallado (chapters 8.6, 8.7)
            % -------------------------------------------------------------------------------------------------------------
            %% INPUT
            % i_ui32mMax: [1] Maximum order of SHE model (equal to lMax)
            % i_dLat:     [1] Latitude of SC in Target Body fixed frame
            % i_dLong:    [1] Longitude of SC in Target Body fixed frame
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
            o_dSHEtrigLUT = zeros(i_ui32mMax+2, 3); % Must be +2 not +1 (check if idSave exceeds pre-allocation to verify).

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

            idSave = uint32(3);

            % Recursion loop
            for mLoop = 2:i_ui32mMax+1

                % Sin recursion
                o_dSHEtrigLUT(idSave, 1) = 2*cosLong * o_dSHEtrigLUT(idSave-uint32(1), 1) - ...
                    o_dSHEtrigLUT(idSave-2, 1);

                % Cosine recursion
                o_dSHEtrigLUT(idSave, 2) = 2*cosLong * o_dSHEtrigLUT(idSave-uint32(1), 2) - ...
                    o_dSHEtrigLUT(idSave-2, 2);

                % mTangent recursion
                o_dSHEtrigLUT(idSave, 3) = o_dSHEtrigLUT(idSave-uint32(1), 3) + tanLat;

                % Increase counter by 1
                idSave = idSave + uint32(1);
            end

            % Check for numerical zeros and insert true zeros
            o_dSHEtrigLUT( (abs(o_dSHEtrigLUT) - eps) < 1.5 * eps ) = 0;

        end

    end

end
