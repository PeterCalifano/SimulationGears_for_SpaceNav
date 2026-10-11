function [dPosVeldt, strAccelInfo] = ...
    EvalRHS_InertialDynOrbit(dxState_IN, dDCMmainAtt_INfromTF, dMainGM, dRefRmain, ...
                            dCoeffSRP, d3rdBodiesGM, dBodyEphemerides, dMainCSlmCoeffCols, ...
                            ui32MaxSHdegree, ui16StatesIdx, dResidualAccel, bIsInEclipse, ...
                            bUseSrpLut, strResponseLut, strSrpData, bIncludeTransverse) %#codegen
%% SIGNATURE
% [dPosVeldt, strAccelInfo] = EvalRHS_InertialDynOrbit(dxState_IN, dDCMmainAtt_INfromTF, ...
%     dMainGM, dRefRmain, dCoeffSRP, d3rdBodiesGM, dBodyEphemerides, dMainCSlmCoeffCols, ...
%     ui32MaxSHdegree, ui16StatesIdx, dResidualAccel, bIsInEclipse, ...
%     bUseSrpLut, strResponseLut, strSrpData, bIncludeTransverse)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Evaluate inertial position/velocity dynamics about the main body. Third-body
% gravity is the physical differential acceleration between the spacecraft and
% the main body. The Sun is always the first entry in d3rdBodiesGM and its
% matching ephemeris is always the first three elements of dBodyEphemerides.
% Select cannonball SRP by default or LUT/bias SRP through the trailing inputs.
% Compose either force here and keep residual acceleration independent.
% Freeze the LUT selection/table for code generation; retain resolved physical
% inputs at runtime. Preserve existing positional calls and report the selected
% SRP acceleration through one model-independent diagnostic field.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dxState_IN           Inertial state [LU; LU/s], optionally with additional states.
% dDCMmainAtt_INfromTF  Main-body-fixed to inertial rotation for harmonics.
% dMainGM              Main-body gravitational parameter [LU^3/s^2].
% dRefRmain            Harmonics reference radius [LU].
% dCoeffSRP            Current-pressure cannonball coefficient [LU/s^2]; default empty.
% d3rdBodiesGM         Sun-first third-body gravitational parameters [LU^3/s^2].
% dBodyEphemerides     Matching packed inertial positions [LU].
% dMainCSlmCoeffCols   Harmonic coefficients; default empty.
% ui32MaxSHdegree      Requested harmonic degree; default empty.
% ui16StatesIdx        Inclusive orbital-state bounds; MATLAB/MEX default is 1:6.
% dResidualAccel       Independent residual acceleration [LU/s^2]; default zero.
% bIsInEclipse         Suppress SRP when true; default false.
% bUseSrpLut           Compile-time LUT selection; default false.
% strResponseLut       Fixed-size numeric LUT; unused with cannonball SRP.
% strSrpData           Resolved numerical inputs for EvalRHS_SRPLutWithBias;
%                      unused with cannonball SRP. Supply no filter indices or modes.
% bIncludeTransverse   Compile-time transverse selection; unused with cannonball SRP.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPosVeldt            Six orbital derivatives [LU/s; LU/s^2].
% strAccelInfo         Optional force components, SRP range and active flag.
%                      Report dAccSRP for the selected cannonball or LUT model [LU/s^2].
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 07-07-2024    Pietro Califano     First version, inherited from EvalRHS_DynLEO for more general orbital
%                                   motion in small bodies environments.
% 08-07-2024    Pietro Califano     Prototype coding completed (SH and SRP implementation). Test required.
% 12-07-2024    Pietro Califano     Function debugging and verification completed.
% 17-08-2024    Pietro Califano     Improved robustness, flexibility for filters, fixed design errors.
% 20-06-2025    Pietro Califano     Major fix: incorrect 3rd bodies acceleration computation (sign)
% 30-04-2026    Pietro Califano, Codex 5.5    Routed cannonball SRP through standalone acceleration kernel.
% 22-07-2026    Pietro Califano, Codex 5.6    Correct the remaining global sign error in direct and indirect
%                                             third-body gravity.
% 23-07-2026    Pietro Califano, Codex        Enforce generated-code ephemeris column-vector indexing.
% 30-09-2026    Pietro Califano, Codex gpt-6  Own LUT/bias selection and force reporting in the orbital RHS.
% 01-10-2026  Pietro Califano, Codex GPT-6  Unify selected SRP acceleration diagnostics.
% 01-10-2026  Pietro Califano, Codex GPT-6  Correct nodal transverse samples and constant inclusion.
% 01-10-2026  Pietro Califano, Codex GPT-6  Name the numeric LUT and acceleration diagnostics.
% 11-10-2026  Pietro Califano, Codex GPT-6  Share gravity kernels while preserving runtime SRP inputs.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% ComputePointMassGravity, ComputeThirdBodyGravity, ComputeCannonballSRP,
% EvalRHS_SRPLutWithBias, EvalExtSphHarmExpInWorldFrame.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dxState_IN
    dDCMmainAtt_INfromTF
    dMainGM
    dRefRmain
    dCoeffSRP           double = []
    d3rdBodiesGM        double = []
    dBodyEphemerides    double = []
    dMainCSlmCoeffCols  double = []
    ui32MaxSHdegree     uint32 = []
    ui16StatesIdx       uint16 = []
    dResidualAccel      double = zeros(3, 1)
    bIsInEclipse        logical = false
    bUseSrpLut (1, 1) logical {coder.mustBeConst} = false
    strResponseLut (1, 1) struct = struct()
    strSrpData (1, 1) struct = struct()
    bIncludeTransverse (1, 1) logical {coder.mustBeConst} = false
end

arguments (Output)
    dPosVeldt (6, 1) double
    strAccelInfo (1, 1) struct
end

% Name the selected fixed-size numeric payload before SRP evaluation.
% Keep the unused legacy input empty and avoid conditionally naming an entry input.
if coder.const(bUseSrpLut)
    strLutForModel = strResponseLut;
    coder.cstructname(strLutForModel, 'SSrpResponseLut');
end

%% INPUT MANAGEMENT
if coder.target('MATLAB') || coder.target('MEX')
    if isempty(ui16StatesIdx)
        % If empty, assume that state is (position, velocity)
        ui16posVelIdx = uint16(1:6);

    else
        ui16posVelIdx = ui16StatesIdx(1, 1):ui16StatesIdx(1, 2 ); % [1 to 6]
    end
else
        ui16posVelIdx = ui16StatesIdx(1, 1):ui16StatesIdx(1, 2 ); % [1 to 6]
end

% Construct local indices
dSunPos_IN = zeros(3,1);
ui8N3rdBodies = uint8(0);

if size(dBodyEphemerides, 1) > 3
    ui8N3rdBodies = uint8(size(dBodyEphemerides, 1) / 3.0) - 1;
end

d3rdBodiesPos_IN = zeros(3, ui8N3rdBodies);
bHasValidEphemerides = false;
bSunPosValid = false;

if ~isempty(dBodyEphemerides)

    bHasValidEphemerides = any(abs(dBodyEphemerides) > eps('single'));

    if bHasValidEphemerides

        dSunPos_IN(:)      = dBodyEphemerides(1:3, 1);
        bSunPosValid = all(isfinite(dSunPos_IN)) && any(abs(dSunPos_IN) > eps('single'));

        % Get number of 3rd bodies other than Sun
        if coder.target("MATLAB") || coder.target("MEX")
            assert(length(d3rdBodiesGM) == ui8N3rdBodies + 1)
        end

        if ui8N3rdBodies > 0
            d3rdBodiesPos_IN(:,:) = reshape( ...
                dBodyEphemerides(4:end, 1), 3, ui8N3rdBodies);
        end

    end
end

%% Function code: Acceleration models computation
% Allocate variables
dAccTot      = coder.nullcopy(zeros(3, 1));
dPosVeldt    = coder.nullcopy(zeros(6, 1));

%% Gravity Main acceleration
% Reuse the central-gravity law of the two-body RHS and analytical Jacobians.
dAccTot(1:3) = ComputePointMassGravity(dxState_IN(ui16posVelIdx(1:3)), dMainGM);

%% Spherical Harmonics acceleration
dAccNonSphr_IN = zeros(3,1);

if ~isempty(dMainCSlmCoeffCols) && any(abs(dDCMmainAtt_INfromTF) > eps('single'), 'all')

    [~, dAccNonSphr_IN] = EvalExtSphHarmExpInWorldFrame(dxState_IN(ui16posVelIdx(1:3)), ...
                                                        dDCMmainAtt_INfromTF, ...
                                                        ui32MaxSHdegree, ...
                                                        dMainCSlmCoeffCols, ...
                                                        dMainGM, ...
                                                        dRefRmain);

end

%% 3rd Body accelerations
dTotAcc3rdBody = zeros(3,1);
dAcc3rdSun     = zeros(3,1);
dPosSunToSC    = zeros(3,1);
dSCdistToSun    = 0.0;
dSRPdistToSun    = 0.0;
bIsSRPActive     = false;

if ~isempty(dBodyEphemerides)
    if bHasValidEphemerides

        % Add up accelerations of all bodies other than the Sun
        if ui8N3rdBodies > 0

            for idB = 1:ui8N3rdBodies

                d3rdBodyGM = d3rdBodiesGM(idB+1);
                d3rdBodyPos_IN = d3rdBodiesPos_IN(1:3, idB);

                if d3rdBodyGM <= 0.0
                    continue;
                end

                if ~(all(isfinite(d3rdBodyPos_IN)) && any(abs(d3rdBodyPos_IN) > eps('single')))
                    if coder.target('MATLAB')
                        warning('Failed to add 3rd body acceleration to RHS! Invalid input.')
                    end
                    continue;
                end

                % Compute 3rd body acceleration and add to total
                dTotAcc3rdBody(:) = dTotAcc3rdBody(1:3) + ComputeThirdBodyGravity( ...
                    dxState_IN(ui16posVelIdx(1:3)), d3rdBodyPos_IN, d3rdBodyGM);
            end

        end

    if bSunPosValid
        % Compute SC position relative to bodies
        dPosSunToSC(:) = dxState_IN(ui16posVelIdx(1:3)) - dSunPos_IN;
        dSCdistToSun = norm(dPosSunToSC);

        if d3rdBodiesGM(1) > 0.0
            dAcc3rdSun(1:3) = ComputeThirdBodyGravity( ...
                dxState_IN(ui16posVelIdx(1:3)), dSunPos_IN, d3rdBodiesGM(1));
        end

    else
        if coder.target('MATLAB')
            if exist('dCoeffSRP', 'var') && any(abs(dCoeffSRP) > eps('single'))
                fprintf('\nWARNING! SRP acceleration computation skipped due to missing Sun ephemerides despite SRP data have been provided!\n')
            end
        end
    end

    end
end

%% Selected SRP acceleration
% Evaluate SRP inside the orbital model; keep its components separate from residuals.
dAccSRP = zeros(3, 1);
if coder.const(bUseSrpLut)
    if bSunPosValid
        % Use the LUT/bias SRP model with the selected fixed-size numeric LUT and resolved inputs.
        dSRPdistToSun = dSCdistToSun;
        if ~bIsInEclipse
            dAccSRP = EvalRHS_SRPLutWithBias(-dPosSunToSC, strSrpData, strLutForModel, bIncludeTransverse);
            bIsSRPActive = any(dAccSRP ~= 0);
        end
    end
elseif ~isempty(dBodyEphemerides) && bSunPosValid && ~isempty(dCoeffSRP)
    % Fixed coefficient cannonball SRP model + bias
    [dAccSRP, dSRPdistToSun, bIsSRPActive] = ComputeCannonballSRP(dPosSunToSC, ...
                                                                dCoeffSRP, ...
                                                                bIsInEclipse);
end

%% Acceleration sum
strAccelInfo = struct();

if nargout > 1
    strAccelInfo.dAccMain           = dAccTot;
    strAccelInfo.dTotAcc3rdBody     = dTotAcc3rdBody;
    strAccelInfo.dAcc3rdSun         = dAcc3rdSun;
    strAccelInfo.dAccSRP            = dAccSRP;
    strAccelInfo.dSRPdistToSun      = dSRPdistToSun;
    strAccelInfo.bIsSRPActive       = bIsSRPActive;
    strAccelInfo.dAccNonSphr_IN     = dAccNonSphr_IN;
    strAccelInfo.dAccPolyhedronPert_IN = zeros(3, 1);
end

dAccTot = dAccTot + dTotAcc3rdBody + dAcc3rdSun + dAccSRP + dAccNonSphr_IN + dResidualAccel;

%% Compute output state time derivative
dPosVeldt(1:6) = [dxState_IN(ui16posVelIdx(4:6));
                   dAccTot];


end
