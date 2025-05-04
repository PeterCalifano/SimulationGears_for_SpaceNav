classdef EnumScenarioName
%% DESCRIPTION
% Enumeration class listing all available scenarios in nav-backend (and by extension, nav-system)
% MATLAB simulation environment.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 17-08-2024        Pietro Califano         Class definition.
% 12-02-2025        Pietro Califano         Add Apophis for RCS1
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
    Eros
    Bennu
    Apophis
    Moon
end

end
