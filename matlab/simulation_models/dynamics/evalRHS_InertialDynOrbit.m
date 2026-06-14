function [dPosVeldt, strAccelInfo] = evalRHS_InertialDynOrbit( dxState_IN, ...
                                                                dDCMmainAtt_INfromTF, ...
                                                                dMainGM, ...
                                                                dRefRmain, ...
                                                                dCoeffSRP, ...
                                                                d3rdBodiesGM, ...
                                                                dBodyEphemerides, ...
                                                                dMainCSlmCoeffCols, ...
                                                                ui32MaxSHdegree, ...
                                                                ui16StatesIdx, ...
                                                                dResidualAccel, ...
                                                                bIsInEclipse) %#codegen
arguments
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
    dResidualAccel      double = zeros(3,1)
    bIsInEclipse        logical = false
end %#codegen
%% PROTOTYPE
% [dPosVeldt, strAccelInfo] = evalRHS_InertialDynOrbit( dxState_IN, ...
%                                                                 dDCMmainAtt_INfromTF, ...
%                                                                 dMainGM, ...
%                                                                 dRefRmain, ...
%                                                                 dCoeffSRP, ...
%                                                                 d3rdBodiesGM, ...
%                                                                 dBodyEphemerides, ...
%                                                                 dMainCSlmCoeffCols, ...
%                                                                 ui32MaxSHdegree, ...
%                                                                 ui16StatesIdx, ...
%                                                                 dResidualAccel) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% ACHTUNG: Sun is always assumed to be the first body in the list of d3rdBodiesGM and processed in this way.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dxState_IN
% dDCMmainAtt_INfromTF
% dMainGM
% dRefRmain
% dCoeffSRP           double = []
% d3rdBodiesGM        double = []
% dBodyEphemerides    double = []
% dMainCSlmCoeffCols  double = []
% ui32MaxSHdegree     uint32 = []
% ui32StatesIdx       uint32 = []
% dResidualAccel      double = zeros(3,1)
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dPosVeldt
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 07-07-2024    Pietro Califano     First version, inherited from evalRHS_DynLEO for more general orbital 
%                                   motion in small bodies environments.
% 08-07-2024    Pietro Califano     Prototype coding completed (SH and SRP implementation). Test required.
% 12-07-2024    Pietro Califano     Function debugging and verification completed.
% 17-08-2024    Pietro Califano     Improved robustness, flexibility for filters, fixed design errors.
% 20-06-2025    Pietro Califano     Major fix: incorrect 3rd bodies acceleration computation (sign)
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------

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
dMainBodyPos_IN = zeros(3,1);
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
            d3rdBodiesPos_IN(:,:) = reshape(dBodyEphemerides(4:end), 3, ui8N3rdBodies); % TODO may require modification, if so, just add a extraction index that moved along column
        end

    end
end

%% Function code: Acceleration models computation
% Allocate variables
dAccTot      = coder.nullcopy(zeros(3, 1));
dPosVeldt    = coder.nullcopy(zeros(6, 1));

% Compute auxiliary variables
dPosNorm = sqrt( dxState_IN(ui16posVelIdx(1))^2 + ...
                 dxState_IN(ui16posVelIdx(2))^2 + ...
                 dxState_IN(ui16posVelIdx(3))^2 );

dPosNorm2 = dPosNorm  * dPosNorm;
dPosNorm3 = dPosNorm2 * dPosNorm;
% dPosNorm4 = dPosNorm3 * dPosNorm;

%% Gravity Main acceleration
dAccTot(1:3) = - (dMainGM/dPosNorm3) * dxState_IN(ui16posVelIdx(1:3));

%% Spherical Harmonics acceleration
dAccNonSphr_IN = zeros(3,1);

if coder.const(not(isempty(dMainCSlmCoeffCols))) && all(dDCMmainAtt_INfromTF ~= 0, 'all')

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

if ~isempty(dBodyEphemerides)
    if bHasValidEphemerides

        % Add up accelerations of all bodies other than the Sun
        if ui8N3rdBodies > 0

            for idB = 1:ui8N3rdBodies

                % Compute position wrt idBth body
                dPos3rdBodiesToSC = zeros(3, 1); % TODO modify this for static sizing
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

                dPos3rdBodiesToSC(:) = dxState_IN(ui16posVelIdx(1:3)) - d3rdBodyPos_IN;

                if coder.target('MATLAB') || coder.target('MEX')
                    assert(any(abs(dPos3rdBodiesToSC) > eps('single')), 'ERROR: distance to 3rd body cannot be near zero!')
                end

                % Compute 3rd body acceleration
                d3rdBodyPosFromMain_IN = d3rdBodyPos_IN - dMainBodyPos_IN;

                dTotAcc3rdBody(:) = dTotAcc3rdBody(1:3) + d3rdBodyGM * ...
                                                 ( dPos3rdBodiesToSC./( norm(dPos3rdBodiesToSC) )^3 + ...
                                                 d3rdBodyPosFromMain_IN./(norm(d3rdBodyPosFromMain_IN)^3) );
            end

        end

        if bSunPosValid
            % Compute SC position relative to bodies
            dPosSunToSC(:) = dxState_IN(ui16posVelIdx(1:3)) - dSunPos_IN;
            dPosSunFromMain_IN = dSunPos_IN - dMainBodyPos_IN;

            dSCdistToSun(:) = norm(dPosSunToSC);

            if coder.target('MATLAB') || coder.target('MEX')
                assert(any(abs(dSunPos_IN) > eps('single')), 'ERROR: Sun position cannot be near zero!')
                assert(any(abs(dSCdistToSun) > eps('single')), 'ERROR: distance to the Sun cannot be zero!')
                assert(any(abs(dPosSunFromMain_IN) > eps('single')), 'ERROR: distance from main body to the Sun cannot be near zero!')
            end

            % DEVNOTE: replace with more accurate formula to handle it in double precision
            % Current solution only bypasses the issue caused by the difference.
            dAuxTerm1 = dPosSunToSC./(dSCdistToSun)^3;
            dAuxTerm2 = dPosSunFromMain_IN./( norm(dPosSunFromMain_IN)^3);

            if all(dAuxTerm1 < 1E-23, 'all') && all(dAuxTerm2 < 1e-23, 'all')
                dAuxTerm3 = zeros(3,1);
            else
                dAuxTerm3 = dAuxTerm1 + dAuxTerm2;
            end

            % Sun 3rd Body acceleration
            if d3rdBodiesGM(1) > 0.0
                dAcc3rdSun(1:3) = d3rdBodiesGM(1) * dAuxTerm3;
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

%% Cannonball SRP acceleration
if ~isempty(dBodyEphemerides) && bSunPosValid && not(bIsInEclipse)
    dAccCannonBallSRP = dCoeffSRP * dPosSunToSC./dSCdistToSun;
else
    dAccCannonBallSRP = zeros(3,1);
end

%% Acceleration sum
strAccelInfo = struct();

if nargout > 1
    strAccelInfo.dAccMain           = dAccTot;
    strAccelInfo.dTotAcc3rdBody     = dTotAcc3rdBody;
    strAccelInfo.dAcc3rdSun         = dAcc3rdSun;
    strAccelInfo.dAccCannonBallSRP  = dAccCannonBallSRP;
    strAccelInfo.dAccNonSphr_IN     = dAccNonSphr_IN;
end

dAccTot = dAccTot + dTotAcc3rdBody + dAcc3rdSun + dAccCannonBallSRP + dAccNonSphr_IN + dResidualAccel;

%% Compute output state time derivative
dPosVeldt(1:6) = [dxState_IN(ui16posVelIdx(4:6));
                   dAccTot];


end
