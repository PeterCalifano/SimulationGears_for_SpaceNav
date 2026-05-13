classdef EnumScenarioName < uint32
%% DESCRIPTION
% Enumeration class listing all available scenarios in nav-backend (and by extension, nav-system) MATLAB simulation environment.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 17-08-2024    Pietro Califano     Class definition.
% 12-02-2025    Pietro Califano     Add Apophis for RCS1
% 30-06-2025    Pietro Califano     Extend to support new version of generation programs.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

enumeration
    Didymos (1)
    Itokawa (2)
    ItokawaModified (3)
    Eros (4)
    Bennu (5)
    Apophis (6)
    ApophisElongated (7)
    Moon (8)
    Mars (9)
    Ceres (10)
    Earth (11)
    FromShape (12)
    NotDefined (13)
end

end
