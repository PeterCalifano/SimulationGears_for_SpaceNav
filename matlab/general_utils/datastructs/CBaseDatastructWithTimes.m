classdef (Abstract) CBaseDatastructWithTimes < CBaseDatastruct
    %% DESCRIPTION
    % Datastruct containing essential information for spacecraft orbit and attitude as sequence of discrete
    % states on a discrete timegrid. The following data are included:
    % REQUIRED
    % TODO
    % OPTIONAL
    % TODO
    % -------------------------------------------------------------------------------------------------------------
    %% CHANGELOG
    % 04-02-2025    Pietro Califano     First prototype implementation
    % -------------------------------------------------------------------------------------------------------------
    %% METHODS
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% PROPERTIES
    % TODO
    % -------------------------------------------------------------------------------------------------------------
    %% DEPENDENCIES
    % [-]
    % -------------------------------------------------------------------------------------------------------------
    %% Future upgrades
    % [-]
    % -------------------------------------------------------------------------------------------------------------

    properties 
        dTimestamps                  (1,:)  double {isnumeric, isvector} = [];
    end

    methods (Access = public)
        function self = CBaseDatastructWithTimes(dTimestamps)
            arguments
                dTimestamps                  (1, :)     double {isnumeric, isvector} = [];

            end
            
            % Store REQUIRED attributes
            self.dTimestamps                = dTimestamps                  ;
        end

        % GETTERS
        function [argout] = queryDataAtTime(self, dTimestamp, charFieldName)
            arguments
                self
                dTimestamp      (1,1) double
                charFieldName   (1,1) string {mustBeA(charFieldName, ['string', 'char'])}
            end

            % TODO: method to query a field at a given timestamp in time grid
            % TODO v2 extend function to query multiple times at once
            
            % Assert attributes validity
            self.assertValidity_;

            % Search for time in timegrid
            dTimeID = find(self.dTimestamps == dTimestamp, 1);
            assert(not(isempty(dTimeID)), 'ERROR: requested timestamp does not exist in timegrid!')
            
                        % TODO assert size of attribute is the same of timegrid

            % TODO index field (must index correct dimension!
            % argout = self.(charFieldName);
        end
    end


    methods (Access = protected)
        function assertValidity_(self)
            assert(not(isempty(self.dTimestamps)), 'ERROR: called query method with empty self.dTimestamps. You MUST specify timestamps to use this data struct.')
            assert(self.dTimestamps(end) > self.dTimestamps(1), 'ERROR: invalid self.dTimestamps. Last timestamp must be greater than first timestamp!')
        end
    end



end
