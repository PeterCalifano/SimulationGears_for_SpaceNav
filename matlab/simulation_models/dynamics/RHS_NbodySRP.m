function dxdt = RHS_NbodySRP(i_dT, i_dxState, i_dMuMain, cellNbodyParams, SRPparams) %#codegen
%% PROTOTYPE
% dxdt = RHS_NbodySRP(t, x, muMain, NbodyParams, SRPparams)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% General purpose orbital dynamics for N bodies + SRP modeled as Cannon
% Ball model. Acceleration term is initialized as unperturbed 2BP. Other
% N-1 bodies accelerations and SRP are added if inputs are provided. SRP
% model assumes inputs uses [m] as Length unit and [s] as Time unit. 
% Body Ephemerides are assumed to be provided with 1 second time step and
% matched with propagation time (starting from t0).
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% t: [1] Time in seconds
% x: [6x1] SC state in inertial frame centred in Main
% muMain: [1] Gravitational parameter of the Main attractor
% NbodyParams: [Nx2] Cell containing: {Grav. Parameter, EPH} of Nth body
% SRPparams: cell containing data required by SRP model
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% dxdt: [6x1] Derivative of the SC orbital state 
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 10-07-2023    Pietro Califano     Function coded and verified
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 1) Upgrade to consider generic timegrids for the Ephemerides
% 2) Add SHE for the desired bodies
% TODO:
% Rework to do: remove cell and replace with array of ephemerides 
% Expand Mu to include all bodies with assert for EPH entries
% Input id indicating which is the Sun (for SRP)
% Replace struct of SRP with arrays
% -------------------------------------------------------------------------------------------------------------

%% Function code
% Note: 1 second time step of EPH is assumed
idt = round(i_dT) + 1;

% Central gravity acceleration (Main)
a2BP = -i_dMuMain/norm(i_dxState(1:3))^3 * i_dxState(1:3);

% Initialize total acceleration as 2BP
aGrav = a2BP;


% N-1 bodies accelerations
if nargin > 3

    % Loop over other N-1 bodies
    for idB = 1:length(cellNbodyParams)

        % Retrieve Nth body position
        R_NthBody = cellNbodyParams{idB, 2}(idt, 1:3)';
        % Compute SC position wrt Nth body
        R_SCwrtNthB = i_dxState(1:3) - R_NthBody;
        % Compute SC distance wrt Nth body
        R_SCwrtNthB_norm = norm(R_SCwrtNthB);

        % Sum acceleration terms
        aGrav = aGrav + cellNbodyParams{idB, 1} * ( R_SCwrtNthB./(R_SCwrtNthB_norm)^3 - R_NthBody./(norm(R_NthBody)^3) );

    end

end

% Cannonball model (SRP)
if nargin > 4

    RSCtoSun = SRPparams.R_SUN(idt, 1:3);
    SCDistToSun = norm(RSCtoSun);

    P_Sun = 1367 * (1/(SCDistToSun/1e9))^2/1e8; % [m]

    aSRP =  - (P_Sun * SRPparams.ReflCoeff * SRPparams.A_SC/SRPparams.mSC) * RSCtoSun./SCDistToSun;

else
    aSRP = [0; 0; 0];
end

% Compute State derivative
dxdt = [i_dxState(4:6);
    aGrav + aSRP];

end