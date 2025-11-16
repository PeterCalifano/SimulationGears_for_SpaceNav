function [] = SaveErrorBacktraceLog(objMException, ...
                                    charOutputFolder, ...
                                    charContext, ...
                                    charRedirectedOutput, ...
                                    bAlsoPrintToStdout)
% Save full error backtrace and optional redirected output to a UTF-8 log.
arguments
    objMException           (1,1) MException {mustBeA(objMException, "MException")}
    charOutputFolder        (1,1) string
    charContext             (1,1) string = "Simulation core failure"
    charRedirectedOutput    (1,1) string = ""
    bAlsoPrintToStdout      (1,1) logical = true
end

% Ensure folder exists
if ~isfolder(charOutputFolder)
    mkdir(charOutputFolder);
end

% Build paths and strings
charTimestamp = string(datetime('now','Format','yyyyMMdd_HHmmss'));
charLogFile   = fullfile(charOutputFolder, "trialErrorLog_" + charTimestamp + ".txt");

charReport = objMException.getReport('extended','hyperlinks','off'); % full backtrace
charHeader = "==== ERROR REPORT ====" + newline + ...
    "Timestamp: " + string(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS Z')) + newline + ...
    "Context:   " + charContext + newline + ...
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
    warning("ERROR: %s. Full report saved to: %s", string(objMException.message), charLogFile);
end

end
