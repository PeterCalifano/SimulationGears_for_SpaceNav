classdef EnumScenarioName
%% DESCRIPTION
% Enumeration class listing all available scenarios in nav-backend (and by extension, nav-system)
% MATLAB simulation environment.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 17-08-2024    Pietro Califano     Class definition.
% 12-02-2025    Pietro Califano     Add Apophis for RCS1
% 30-06-2025    Pietro Califano     Extend to support new version of generation programs.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% 1) Add all scenarios that have been tested
% -------------------------------------------------------------------------------------------------------------
%% Function code

enumeration
    Didymos
    Itokawa
    ItokawaModified
    Eros
    Bennu
    Apophis
    ApophisElongated
    Moon
    Mars
    Ceres
    NotDefined
    Earth
end

end
