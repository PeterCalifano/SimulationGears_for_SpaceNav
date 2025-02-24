function [dPosVeldt, strAccelInfo] = evalRHS_InertialDynOrbit( ...
    dxState_IN, ...
    dDCMmainAtt_INfromTF, ...
    dMainGM, ...
    dRefRmain, ...
    dCoeffSRP, ...
    d3rdBodiesGM, ...
    dBodyEphemerides, ...
    dMainCSlmCoeffCols, ...
    ui32MaxSHdegree, ...
    ui32StatesIdx, ...
    dResidualAccel) %#codegen
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
    ui32StatesIdx       uint32 = []
    dResidualAccel      double = zeros(3,1)
end %#codegen
%% PROTOTYPE
% [dPosVeldt, strAccelInfo] = evalRHS_InertialDynOrbit( ...
%     dxState_IN, ...
%     dDCMmainAtt_INfromTF, ...
%     dMainGM, ...
%     dRefRmain, ...
%     dCoeffSRP, ...
%     d3rdBodiesGM, ...
%     dBodyEphemerides, ...
%     dMainCSlmCoeffCols, ...
%     ui32MaxSHdegree, ...
%     ui32StatesIdx, ...
%     dResidualAccel) %#codegen
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% NOTE: Code generation not tested. It is likely that some modifications in how memory and variables are
% handled by this function will be required to do it.
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
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% TODO modify for static-sized evaluation
% -------------------------------------------------------------------------------------------------------------

%% INPUT MANAGEMENT

if isempty(ui32StatesIdx)
    % If empty, assume that state is (position, velocity)
    ui8posVelIdx = uint16(1:6);

else
    ui8posVelIdx = uint8( ui32StatesIdx(1, 1):ui32StatesIdx(1, 2) ); % [1 to 6]
end

% Construct local indices
dSunPos_IN = zeros(3,1);

if ~isempty(dBodyEphemerides)

    if any(dBodyEphemerides > 0)

        dSunPos_IN(:)      = dBodyEphemerides(1:3, 1);

        % Get number of 3rd bodies other than Sun
        ui8N3rdBodies = uint8(size(dBodyEphemerides, 2)) - 1;

        if coder.target("MATLAB") || coder.target("MEX")
            assert(length(d3rdBodiesGM) == ui8N3rdBodies + 1)
        end

        if ui8N3rdBodies > 0
            d3rdBodiesPos_IN = reshape(dBodyEphemerides(4:end), 3, ui8N3rdBodies); % TODO may require modification, if so, just add a extraction index that moved along column
        else
            d3rdBodiesPos_IN = [];
        end

    end
else
    ui8N3rdBodies = 0;
end

%% Function code: Acceleration models computation
% Allocate variables
dAccTot = coder.nullcopy(zeros(3, 1));
dPosVeldt = coder.nullcopy(zeros(6, 1));

% Compute auxiliary variables
dPosNorm = sqrt( dxState_IN(ui8posVelIdx(1))^2 + ...
                 dxState_IN(ui8posVelIdx(2))^2 + ...
                 dxState_IN(ui8posVelIdx(3))^2 );

dPosNorm2 = dPosNorm  * dPosNorm;
dPosNorm3 = dPosNorm2 * dPosNorm;
% dPosNorm4 = dPosNorm3 * dPosNorm;

% Gravity Main acceleration
dAccTot(1:3) = - (dMainGM/dPosNorm3) * dxState_IN(ui8posVelIdx(1:3));

%% Spherical Harmonics acceleration
dAccNonSphr_IN = zeros(3,1);

if not(isempty(dMainCSlmCoeffCols)) && all(dDCMmainAtt_INfromTF ~= 0, 'all')
    
    % Rotate inertial position to target frame
    dxPos_TB = dDCMmainAtt_INfromTF' * dxState_IN(ui8posVelIdx(1:3));
    % Compute Non-spherical acceleration in target frame

    dAccNonSphr_TB = ExtSHE_AccTB(dxPos_TB, ui32MaxSHdegree, ...
        dMainCSlmCoeffCols, dMainGM, dRefRmain); 

    % Rotate Non-spherical acceleration to inertial frame
    dAccNonSphr_IN(:) = dDCMmainAtt_INfromTF * dAccNonSphr_TB;
end

%% 3rd Body accelerations
dTotAcc3rdBody = zeros(3,1);
dAcc3rdSun     = zeros(3,1);

if ~isempty(dBodyEphemerides)
    if any(dBodyEphemerides > 0)

        % Add up accelerations of all bodies other than the Sun
        if ui8N3rdBodies > 0

            for idB = 1:ui8N3rdBodies

                % Compute position wrt idBth body
                dPos3rdBodiesToSC = zeros(3, 1); % TODO modify this for static sizing
                dPos3rdBodiesToSC(:) = dxState_IN(ui8posVelIdx(1:3)) - d3rdBodiesPos_IN(1:3, idB);

                % Compute 3rd body acceleration
                dTotAcc3rdBody(:) = dTotAcc3rdBody(1:3) + d3rdBodiesGM(idB+1) * ...
                                                 ( dPos3rdBodiesToSC./( norm(dPos3rdBodiesToSC) )^3 - ...
                                                 d3rdBodiesPos_IN(:, idB)./(norm(d3rdBodiesPos_IN(:, idB))^3) );
            end

        end

        % Compute SC position relative to bodies
        dPosSunToSC  = dxState_IN(ui8posVelIdx(1:3)) - dSunPos_IN;
        SCdistToSun = norm(dPosSunToSC);

        % DEVNOTE: replace with more accurate formula to handle it in double precision
        % Current solution only bypasses the issue caused by the difference.
        dAuxTerm1 = dPosSunToSC./(SCdistToSun)^3;
        dAuxTerm2 = dSunPos_IN./( norm(dSunPos_IN)^3);

        if all(dAuxTerm1 < eps, 'all') && all(dAuxTerm2 < eps, 'all')
            dAuxTerm3 = zeros(3,1);
        else
            dAuxTerm3 = dAuxTerm1 -  dAuxTerm2;
        end

        % Sun 3rd Body acceleration
        dAcc3rdSun(1:3) = d3rdBodiesGM(1) * dAuxTerm3;

    else
        if exist('dCoeffSRP', 'var')
            fprintf('\nWARNING! SRP acceleration computation skipped due to missing Sun ephemerides despite SRP data have been provided!\n')
        end
    end

end

% Cannonball SRP acceleration
if ~isempty(dBodyEphemerides) % TODO: need a way to disable this --> factory pattern for classes?
    dAccCannonBallSRP = dCoeffSRP * dPosSunToSC./SCdistToSun;
else
    dAccCannonBallSRP = zeros(3,1);
end

%% Acceleration sum
strAccelInfo = struct();

if nargout > 1
    strAccelInfo.dAccMain = dAccTot;
    strAccelInfo.dTotAcc3rdBody = dTotAcc3rdBody;
    strAccelInfo.dAcc3rdSun = dAcc3rdSun;
    strAccelInfo.dAccCannonBallSRP = dAccCannonBallSRP;
    strAccelInfo.dAccNonSphr_IN = dAccNonSphr_IN;
end

dAccTot = dAccTot + dTotAcc3rdBody + dAcc3rdSun + dAccCannonBallSRP + dAccNonSphr_IN + dResidualAccel;

%% Compute output state time derivative
dPosVeldt(1:6) = [dxState_IN(ui8posVelIdx(4:6));
                   dAccTot];


end
