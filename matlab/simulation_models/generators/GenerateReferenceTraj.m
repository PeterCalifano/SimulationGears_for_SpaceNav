function objDataset = GenerateReferenceTraj(objDataset, ...
                                            enumTargetName_Dyn, ...
                                            objScenarioConfig, ...
                                            bDEBUG_MODE)
arguments
    objDataset         (1,1) {mustBeA(objDataset, "SReferenceMissionDesign")}
    enumTargetName_Dyn (1,1) {mustBeA(enumTargetName_Dyn, ["EnumScenarioName", "string", "char"])}
    objScenarioConfig  (1,1) {mustBeA(objScenarioConfig, "SScenarioConfiguration")}
    bDEBUG_MODE        (1,1) logical = false
end
%% PROTOTYPE
% TODO
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
%  
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% in1 [dim] description
% Name1                     []
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% out1 [dim] description
% Name1                     []
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 03-05-2024    Pietro Califano     Function adapted from script.
% 08-07-2024    Pietro Califano     Update to support more complex dynamics
% 19-07-2024    Pietro Califano     Update of interfaces and processing
% 06-07-2025    Pietro Califano     Major rework to use new SimGears tools
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Future upgrades
% [-]
% -------------------------------------------------------------------------------------------------------------
%% Build scenario
% Build data scenario generator class
% ACHTUNG: timgegrid MUST be in Ephemeris Time (ET, for CSPICE)
[strDynParams, strMainBodyRefData] = DefineEnvironmentProperties(objScenarioConfig.dEphemeridesTimegrid, ...
                                                                 enumTargetName_Dyn, ...
                                                                 objScenarioConfig.enumWorldFrame, ...
                                                                 "objDataset", objDataset);

% Build ephemerides
ui32EphemerisPolyDeg = 15;
ui32AttitudePolyDeg  = 25;

[strDynParams, strMainBodyRefData] = EphemeridesDataFactory(objScenarioConfig.dEphemeridesTimegrid, ...
                                                            ui32EphemerisPolyDeg,...
                                                            ui32AttitudePolyDeg, ...
                                                            strDynParams, ...
                                                            strMainBodyRefData, ...
                                                            "bGroundTruthEphemerides", true, ...
                                                            "bEnableInterpValidation", true);

% Call scenario generator to build the actual objDataset
objScenarioGenerator = CScenarioGenerator(  [objDataset.dPosSC_W(:,objScenarioConfig.dInitialTimeID)
                                            objDataset.dVelSC_W(:,objScenarioConfig.dInitialTimeID)], ...
                                            objScenarioConfig.dTimestamps, ...
                                            strDynParams, ...
                                            "bProvideAccelerationData", true, ...
                                            "enumWorldFrameName", objDataset.enumWorldFrame);

% Generate data
objDataset = objScenarioGenerator.generateData();

if bDEBUG_MODE
    % PLOT REFERENCE
    figure;
    plot3(objDataset.dPosSC_W(:, 1), objDataset.dPosSC_W(:, 2), objDataset.dPosSC_W(:, 3), 'k-', 'LineWidth', 1.05);
    hold on;
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
end


end
