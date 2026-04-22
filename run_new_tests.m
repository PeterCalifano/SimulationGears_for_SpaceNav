charRootDir = getScriptRootDir_();

run(fullfile(charRootDir, 'matlab', 'SetupSimGears.m'));
addpath(genpath(fullfile(charRootDir, 'tests', 'matlab')));
addpath(genpath(fullfile(charRootDir, 'lib', 'MathCore_for_SpaceNav', 'tests', 'matlab')));

testFiles = {
    'tests/matlab/simulation_models/dynamics/testRHS_2BPwSTM'
    'tests/matlab/simulation_models/dynamics/testPropagateClohessyWiltshireAnalytical'
    'tests/matlab/simulation_models/dynamics/testEvalRHS_GaussPlanetary'
    'tests/matlab/simulation_models/measurements/cameras/testPinholeProjectArrayHP'
    'tests/matlab/simulation_models/measurements/common/testRayTriangleIntersection'
    'tests/matlab/simulation_models/accelerations/testApplyImpulsiveDeltaVError'
    'lib/MathCore_for_SpaceNav/tests/matlab/astrodynamics/testKepl2rv'
};

nSuites = numel(testFiles);
nPassed = 0;
nFailed = 0;
nIncomplete = 0;
nSuiteErrors = 0;
nResults = 0;
tRun = tic;
objRunner = matlab.unittest.TestRunner.withNoPlugins;

fprintf('Running %d test files\n', nSuites);

for iSuite = 1:nSuites
    charSuite = testFiles{iSuite};
    [~, charSuiteName] = fileparts(charSuite);
    tSuite = tic;

    fprintf('\n[%d/%d] %s\n', iSuite, nSuites, charSuiteName);

    try
        objSuite = testsuite(charSuite);
        res = objRunner.run(objSuite);

        nSuiteResults = numel(res);
        nSuitePassed = sum([res.Passed]);
        nSuiteFailed = sum([res.Failed]);
        nSuiteIncomplete = sum([res.Incomplete]);

        nResults = nResults + nSuiteResults;
        nPassed = nPassed + nSuitePassed;
        nFailed = nFailed + nSuiteFailed;
        nIncomplete = nIncomplete + nSuiteIncomplete;

        fprintf('  %d passed', nSuitePassed);
        if nSuiteFailed > 0
            fprintf(', %d failed', nSuiteFailed);
        end
        if nSuiteIncomplete > 0
            fprintf(', %d incomplete', nSuiteIncomplete);
        end
        fprintf(' (%.2f s)\n', toc(tSuite));

        if nSuiteFailed == 0 && nSuiteIncomplete == 0
            continue
        end

        for iResult = 1:nSuiteResults
            objResult = res(iResult);
            if objResult.Passed
                continue
            end

            printResultIssue_(objResult);
        end
    catch ME
        nSuiteErrors = nSuiteErrors + 1;
        fprintf(2, '  error loading suite: %s\n', firstLine_(ME.message));
    end
end

fprintf('\nSummary\n');
fprintf('  Test files:   %d\n', nSuites);
fprintf('  Test results: %d\n', nResults);
fprintf('  Passed:       %d\n', nPassed);
fprintf('  Failed:       %d\n', nFailed);
fprintf('  Incomplete:   %d\n', nIncomplete);
fprintf('  Suite errors: %d\n', nSuiteErrors);
fprintf('  Elapsed:      %.2f s\n', toc(tRun));

if nFailed == 0 && nIncomplete == 0 && nSuiteErrors == 0
    fprintf('\nAll requested tests passed.\n');
else
    fprintf(2, '\nTest run completed with issues.\n');
end

function charRootDir = getScriptRootDir_()
charScriptPath = mfilename('fullpath');

if isempty(charScriptPath)
    charScriptPath = which('run_new_tests');
end

if isempty(charScriptPath)
    charRootDir = pwd;
else
    charRootDir = fileparts(charScriptPath);
end
end

function printResultIssue_(objResult)
if objResult.Incomplete
    charStatus = 'INCOMPLETE';
elseif objResult.Errored
    charStatus = 'ERROR';
else
    charStatus = 'FAIL';
end

fprintf(2, '  %s %s\n', charStatus, objResult.Name);

cellDiagLines = getDiagnosticPreview_(objResult);
for iLine = 1:numel(cellDiagLines)
    fprintf(2, '    %s\n', cellDiagLines{iLine});
end
end

function cellDiagLines = getDiagnosticPreview_(objResult)
charText = '';

try
    charText = char(string(objResult.Details.DiagnosticRecord.Report));
catch
end

charText = strtrim(regexprep(charText, '\r\n?', '\n'));

if isempty(charText)
    cellDiagLines = {'No diagnostic details available.'};
    return
end

cellDiagLines = regexp(charText, '\n', 'split');
cellDiagLines = cellfun(@strtrim, cellDiagLines, 'UniformOutput', false);
cellDiagLines = cellDiagLines(~cellfun('isempty', cellDiagLines));

if isempty(cellDiagLines)
    cellDiagLines = {'No diagnostic details available.'};
    return
end

if numel(cellDiagLines) > 3
    cellDiagLines = cellDiagLines(1:3);
    cellDiagLines{end + 1} = '...';
end
end

function charLine = firstLine_(charText)
cellLines = regexp(char(charText), '\r\n?|\n', 'split');
cellLines = cellLines(~cellfun('isempty', cellLines));

if isempty(cellLines)
    charLine = '';
else
    charLine = strtrim(cellLines{1});
end
end
