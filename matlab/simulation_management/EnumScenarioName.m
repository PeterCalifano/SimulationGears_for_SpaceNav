classdef EnumScenarioName < uint32
%% DESCRIPTION
% Enumeration class listing all SimulationGears scenario identifiers.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 17-08-2024    Pietro Califano     Class definition.
% 12-02-2025    Pietro Califano     Add Apophis for RCS1
% 30-06-2025    Pietro Califano     Extend to support new version of generation programs.
% 01-07-2026    Pietro Califano     Add first-class tagged small-body scenarios.
% -------------------------------------------------------------------------------------------------------------
%% ENUMERATIONS
% Didymos, Itokawa, Eros, Bennu, Apophis, ApophisElongated, Moon, Mars,
% Ceres, Earth, FromShape, NotDefined, Arrokoth, Comet67P, Toutatis.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Function code

enumeration
    Didymos (1)
    Itokawa (2)
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
    Arrokoth (14)
    Comet67P (15)
    Toutatis (16)
end

end
