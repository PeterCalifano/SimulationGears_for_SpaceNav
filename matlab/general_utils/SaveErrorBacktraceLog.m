function [] = SaveErrorBacktraceLog(objMException, ...
                                    charOutputFolder, ...
                                    charContext, ...
                                    charRedirectedOutput, ...
                                    bAlsoPrintToStdout, ...
                                    dElapsedSeconds, ...
                                    dSimulationTimestamp, ...
                                    charSimulationTimeSource)
% Save full error backtrace and optional redirected output to a UTF-8 log.
arguments
    objMException           (1,1) MException {mustBeA(objMException, "MException")}
    charOutputFolder        (1,1) string
    charContext             (1,1) string = "Simulation core failure"
    charRedirectedOutput    (1,1) string = ""
    bAlsoPrintToStdout      (1,1) logical = true
    dElapsedSeconds         (1,1) double = NaN
    dSimulationTimestamp    (1,1) double = NaN
    charSimulationTimeSource (1,1) string = ""
end

% Ensure folder exists
if ~isfolder(charOutputFolder)
    mkdir(charOutputFolder);
end

% Build paths and strings
charTimestamp = string(datetime('now','Format','yyyyMMdd_HHmmss'));
charLogFile   = fullfile(charOutputFolder, "trialErrorLog_" + charTimestamp + ".txt");

charReport = objMException.getReport('extended','hyperlinks','off'); % full backtrace
charErrorIdentifier = string(objMException.identifier);
if strlength(strtrim(charErrorIdentifier)) == 0
    charErrorIdentifier = "<no-id>";
end

charTopFrame = "<no stack>";
if ~isempty(objMException.stack)
    strTopFrame = objMException.stack(1);
    charTopFrame = sprintf('%s (line %d)', strTopFrame.name, strTopFrame.line);
end

charElapsedSummary = FormatTimingValue_(dElapsedSeconds, "s");
charSimTimestampSummary = FormatTimingValue_(dSimulationTimestamp, "s");
if strlength(strtrim(charSimulationTimeSource)) > 0 && charSimTimestampSummary ~= "<unavailable>"
    charSimTimestampSummary = charSimTimestampSummary + " (" + charSimulationTimeSource + ")";
end

charHeader = "============== ERROR REPORT ==============" + newline + ...
    "Timestamp: " + string(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS Z')) + newline + ...
    "Context:   " + charContext + newline + ...
    "Error ID:  " + charErrorIdentifier + newline + ...
    "Top frame: " + charTopFrame + newline + ...
    "Run time:  " + charElapsedSummary + newline + ...
    "Sim time:  " + charSimTimestampSummary + newline + ...
    "MATLAB:    " + version + " (" + computer + ")" + newline + ...
    "PID:       " + string(feature('getpid')) + newline + ...
    "CWD:       " + string(pwd) + newline + ...
    "======================" + newline + newline;

% Write file (UTF-8)
[fid, msg] = fopen(charLogFile,'w','n','UTF-8');
if fid < 0
    warning("Failed to open log file: %s", msg);
    return
end

cleanupObj = onCleanup(@() fclose(fid));

fprintf(fid, '%s', charHeader);
fprintf(fid, '%s', charReport);

if strlength(strtrim(charRedirectedOutput)) > 0
    fprintf(fid, '\n==== REDIRECTED OUTPUT ====\n%s\n', charRedirectedOutput);
end

% Optional stdout echo
if bAlsoPrintToStdout
    fprintf(2, 'ERROR [%s]: %s\n', char(charErrorIdentifier), char(string(objMException.message)));
    fprintf(2, 'Top frame: %s\n', char(charTopFrame));
    fprintf(2, 'Run time: %s\n', char(charElapsedSummary));
    fprintf(2, 'Simulation time at failure: %s\n', char(charSimTimestampSummary));
    fprintf(2, 'Full report saved to: %s\n', char(charLogFile));
end

end

% Helper function to format timing values
function charSummary = FormatTimingValue_(dValue, charUnits)
if isfinite(dValue)
    charSummary = string(sprintf('%.6f %s', dValue, char(charUnits)));
else
    charSummary = "<unavailable>";
end
end
