function [o_dPosVeldt, strAccelInfo] = evalRHS_DynOrbit( ...
    i_dxState_IN, ...
    i_dDCMmainAtt_INfromTF, ...
    i_dMainGM, ...
    i_dRefRmain, ...
    i_dCoeffSRP, ...
    i_d3rdBodiesGM, ...
    i_dBodyEphemeris, ...
    i_dMainCSlmCoeffCols, ...
    i_ui32MaxSHdegree, ...
    i_ui32StatesIdx) %#codegen
arguments
    i_dxState_IN
    i_dDCMmainAtt_INfromTF
    i_dMainGM
    i_dRefRmain
    i_dCoeffSRP           double = []
    i_d3rdBodiesGM        double = []
    i_dBodyEphemeris      double = []
    i_dMainCSlmCoeffCols  double = []
    i_ui32MaxSHdegree     uint32 = []
    i_ui32StatesIdx       uint32 = []
end %#codegen
%% PROTOTYPE
% o_dPosVeldt = evalRHS_DynOrbit( ...
%     i_dxState_IN, ...
%     i_dDCMmainAtt_INfromTF, ...
%     i_dMainGM, ...
%     i_dRefRmain, ...
%     i_dCoeffSRP, ...
%     i_d3rdBodiesGM, ...
%     i_dBodyEphemeris, ...
%     i_dMainCSlmCoeffCols, ...
%     ui16StatesIdx)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% NOTE: Code generation not tested. It is likely that some modifications in how memory and variables are
% handled by this function will be required to do it.
% ACHTUNG: Sun is always assumed to be the first body in the list of i_d3rdBodiesGM and processed in this way.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% i_dxState_IN
% i_dDCMmainAtt_INfromTF
% i_dMainGM
% i_dRefRmain
% i_dCoeffSRP           double = []
% i_d3rdBodiesGM        double = []
% i_dBodyEphemeris      double = []
% i_dMainCSlmCoeffCols  double = []
% i_ui16StatesIdx         uin16  = []
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% o_dPosVeldt
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
% [-]
% -------------------------------------------------------------------------------------------------------------

%% INPUT MANAGEMENT
if isempty(i_ui32StatesIdx)
    % If empty, assume that state is (position, velocity)
    posVelIdx = uint16(1:6);

else
    posVelIdx = uint8( i_ui32StatesIdx(1, 1):i_ui32StatesIdx(1, 2) ); % [1 to 6]
end

% Construct local indices

if ~isempty(i_dBodyEphemeris)

    dSunPos_IN      = i_dBodyEphemeris(1:3, 1);

    % Get number of 3rd bodies other than Sun
    N3rdBodies = size(i_dBodyEphemeris, 2) - 1;

    assert(length(i_d3rdBodiesGM) == N3rdBodies + 1)

    if N3rdBodies > 0
        d3rdBodiesPos_IN = reshape(i_dBodyEphemeris(4:end), 3, N3rdBodies);
    else
        d3rdBodiesPos_IN = [];
    end
else
    N3rdBodies = 0;
end

%% Function code: Acceleration models computation
% Allocate variables
dAccTot = coder.nullcopy(zeros(3, 1));
o_dPosVeldt = coder.nullcopy(zeros(6, 1));

% Compute auxiliary variables
dPosNorm = sqrt( i_dxState_IN(posVelIdx(1))^2 + ...
                 i_dxState_IN(posVelIdx(2))^2 + ...
                 i_dxState_IN(posVelIdx(3))^2 );

dPosNorm2 = dPosNorm  * dPosNorm;
dPosNorm3 = dPosNorm2 * dPosNorm;
% dPosNorm4 = dPosNorm3 * dPosNorm;

% Gravity Main acceleration
dAccTot(1:3) = - (i_dMainGM/dPosNorm3) *i_dxState_IN(posVelIdx(1:3));

% Spherical Harmonics acceleration

if not(isempty(i_dMainCSlmCoeffCols))
    
    % Rotate inertial position to target frame
    dxPos_TB = i_dDCMmainAtt_INfromTF' * i_dxState_IN(posVelIdx(1:3));
    % Compute Non-spherical acceleration in target frame

    dAccNonSphr_TB = ExtSHE_AccTB(dxPos_TB, i_ui32MaxSHdegree, ...
        i_dMainCSlmCoeffCols, i_dMainGM, i_dRefRmain);

    % Rotate Non-spherical acceleration to inertial frame
    dAccNonSphr_IN = i_dDCMmainAtt_INfromTF * dAccNonSphr_TB;
else
    dAccNonSphr_IN = zeros(3,1);
end

% 3rd Body accelerations
dTotAcc3rdBody = zeros(3,1);
dAcc3rdSun     = zeros(3,1);

if ~isempty(i_dBodyEphemeris)
    
    % Add up accelerations of all bodies other than the Sun
    if N3rdBodies > 0
        dPos3rdBodiesToSC = zeros(3, N3rdBodies);
        dPos3rdBodiesToSC(1:3, :) = i_dxState_IN(posVelIdx(1:3)) - d3rdBodiesPos_IN;  % MODIFY to avoid extraction

        for idB = 1:N3rdBodies
            dTotAcc3rdBody(1:3) = dTotAcc3rdBody(1:3) + i_d3rdBodiesGM(idB+1) * ...
                ( dPos3rdBodiesToSC(:, idB)./(norm(dPos3rdBodiesToSC(:, idB)))^3 - ...
                d3rdBodiesPos_IN(:, idB)./(norm(d3rdBodiesPos_IN(:, idB))^3) );
        end

    end

    % Compute SC position relative to bodies
    dPosSunToSC  = i_dxState_IN(posVelIdx(1:3)) - dSunPos_IN;
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
    dAcc3rdSun(1:3) = i_d3rdBodiesGM(1) * dAuxTerm3;

else
    if exist('i_dCoeffSRP', 'var')
        fprintf('\nWARNING! SRP acceleration computation skipped due to missing Sun ephemerides despite SRP data have been provided!\n')
    end
end

% Cannonball SRP acceleration
if ~isempty(i_dBodyEphemeris) % TODO: need a way to disable this --> factory pattern for classes?
    dAccCannonBallSRP = i_dCoeffSRP * dPosSunToSC./SCdistToSun;
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

dAccTot = dAccTot + dTotAcc3rdBody + dAcc3rdSun + dAccCannonBallSRP + dAccNonSphr_IN;

%% Compute output state time derivative
o_dPosVeldt(1:6) = [i_dxState_IN(posVelIdx(4:6));
                        dAccTot];


end
