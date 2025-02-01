function [o_dSRPaccel_IN, o_dSRPtorque_SCB, o_dPanelsCosSPA] = PanelModelSRP(i_dDirSCtoSun_SCB, ...
    i_dqSCBwrtIN, ...
    i_dMassSC, ...
    i_dCoMpos_SCB, ...
    i_dSolarPressure, ...
    i_dSCpanelsArea, ...
    i_dDiffSpecPanelsCoeffs,...
    i_dPanelsNormals_SCB, ...
    i_dPanelsPressCentre_SCB)
%% PROTOTYPE
% [o_dSRPaccel_IN, o_dForcePerPanel_SCB, o_dPanelsSunPhaseAngle] = PanelModelSRPaccel(i_dDirSCtoSun_SCB, ...
%     i_dqSCBwrtIN, ...
%     i_dMassSC, ...
%     i_dCoMpos_SCB, ...
%     i_dSolarPressure, ...
%     i_dSCpanelsArea, ...
%     i_dDiffSpecPanelsCoeffs,...
%     i_dPanelsNormals_SCB, ...
%     i_dPanelsPressCentre_SCB)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% What the function does
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% in1 [dim] description
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% out1 [dim] description
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 13-12-2023    Pietro Califano    First prototype. Validation pending.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 1) Add selector of quaternion convention
% 2) Add extension to better handle solar arrays motion
% -------------------------------------------------------------------------------------------------------------
%% Function code
% i_dLightSpeed = 299792458 * 1E3; % [km/s]

% Get number of panels
Npanels = uint8(length(i_dSCpanelsArea));

% Input consistency asserts
assert(length(i_dSCpanelsArea) == Npanels)
assert(size(i_dDiffSpecPanelsCoeffs, 1) == Npanels)
assert(size(i_dPanelsNormals_SCB, 2) == Npanels);
assert(size(i_dPanelsPressCentre_SCB, 2) == Npanels);

assert(size(i_dqSCBwrtIN, 1) == 4, 'Input quaternion must be a column vector [4x1].')

% Initialize SRP force vector in SCB
dForcePerPanel_SCB = zeros(3, Npanels);
o_dSRPtorque_SCB = coder.nullcopy(zeros(3, 1));
o_dSRPaccel_IN = coder.nullcopy(zeros(3,1));
o_dPanelsCosSPA = coder.nullcopy(zeros(Npanels, 1));

% Force computation loop
for idP = 1:Npanels

    % Compute cosine of Sun Phase Angle for idP panel
    o_dPanelsCosSPA(idP) = dot(i_dPanelsNormals_SCB(:, idP), i_dDirSCtoSun_SCB);

    %     if o_dPanelsCosSPA(idP) <= 1.570796326794897 % No contribution if angle >= 90°

    %% DEVNOTE: check condition. What is happening if cosine < 0? 
    % Need to separate "free" panels from body panels.
    if o_dPanelsCosSPA(idP) > 0 % No contribution if angle >= 90°

        % Compute force acting on the idP panel in SCB

        % TODO: CHECK FORMULA BREAKING DOWN COMPONENTS
        dForcePerPanel_SCB(:, idP) = - i_dSolarPressure * i_dSCpanelsArea(idP) * o_dPanelsCosSPA(idP) * ...
            ( 2*( i_dDiffSpecPanelsCoeffs(idP, 1)/3 + i_dDiffSpecPanelsCoeffs(idP, 2) * o_dPanelsCosSPA(idP) ) * i_dPanelsNormals_SCB(:, idP) + ...
            (1 - i_dDiffSpecPanelsCoeffs(idP, 2) ) * i_dDirSCtoSun_SCB );
        
        % Compute torque as cross product of PressCentrePos and
        % dForcePerPanel_SCB (by definition of torque)
        o_dSRPtorque_SCB = o_dSRPtorque_SCB + cross(i_dPanelsPressCentre_SCB(:, idP) - i_dCoMpos_SCB, ...
                           dForcePerPanel_SCB(:, idP));

    end

end

% Compute total SRP acceleration in SCB
dSRPaccel_SCB = sum(dForcePerPanel_SCB, 2)/i_dMassSC;

% Rotate acceleration from SCB to Global frame
% Equivalent to A(q)*vec2rotate;
tmp = qCross( qCross( i_dqSCBwrtIN, [dSRPaccel_SCB; 0] ), [-i_dqSCBwrtIN(1:3); i_dqSCBwrtIN(4)] );
o_dSRPaccel_IN(1:3) = tmp(1:3);


%% LOCAL FUNCTIONS

    function q1xq2 = qCross(q1, q2) %#codegen
        %% PROTOTYPE
        % q1xq2 = qCross(q1, q2)
        % -------------------------------------------------------------------------------------------------------------
        %% DESCRIPTION
        % Computes the quaternion product between q1 and q2 using matrix Psi(q1)
        % operation: [q1 x] = [Psi(q) q]. Quaternion convention: JPL (qv, qs)
        % REFERENCE:
        % 1) Fundamentals of Spacecraft Attitude Determination and Control, Markley
        % Crassidis, 2014. Section 2, page 53.
        % -------------------------------------------------------------------------------------------------------------
        %% INPUT
        % q1: [4x1] Quaternion 1 (left side of the quat. product)
        % q2: [4x1] Quaternion 2 (right side of the quat. product)
        % -------------------------------------------------------------------------------------------------------------
        %% OUTPUT
        % q1xq2: [4x1] Quaternion cross product between q1 and q2
        % -------------------------------------------------------------------------------------------------------------
        %% CHANGELOG
        % 05-09-2023    Pietro Califano   Coded and validated for std quat. lib.
        % -------------------------------------------------------------------------------------------------------------
        %% DEPENDENCIES
        % [-]
        % -------------------------------------------------------------------------------------------------------------

        q1xq2 = [q1(4) q1(3) -q1(2) q1(1); ...
            -q1(3) q1(4) q1(1) q1(2); ...
            q1(2) -q1(1) q1(4) q1(3); ...
            -q1(1) -q1(2) -q1(3) q1(4)] * q2;

    end
end
