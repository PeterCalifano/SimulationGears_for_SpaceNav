function testYamlBatch()
%TESTYAMLBATCH  Verify yaml + SnakeYAML works inside a batch job.
%
%  - Discovers yaml.dump and snakeyaml-1.30.jar on the client.
%  - Submits a batch job that:
%       * adds the jar to the Java class path in the worker
%       * calls yaml.dump on a small struct
%       * prints the YAML to the worker diary
%  - Prints the worker diary when finished.

    %% 1) Discover yaml.dump and the SnakeYAML jar on the client
    yamlDumpPath = which('yaml.dump');
    assert(~isempty(yamlDumpPath), ...
        'yaml.dump not found on path. Add the folder containing +yaml.');

    yamlPkgFolder = fileparts(yamlDumpPath);  % .../+yaml
    charSnakeYamlJar = fullfile(yamlPkgFolder, 'snakeyaml', 'snakeyaml-1.30.jar');

    assert(exist(charSnakeYamlJar, 'file') == 2, ...
        'Snakeyaml JAR not found at %s', charSnakeYamlJar);

    fprintf('Client: yaml.dump at    %s\n', yamlDumpPath);
    fprintf('Client: SnakeYAML JAR at %s\n', charSnakeYamlJar);

    %% 2) Choose a process-based cluster
    c = parcluster('local');   % adjust if you use a different profile
    % If your default "local" is threads-only, switch to your Processes profile here.
    fprintf('Using cluster: %s (Type: %s)\n', c.Profile, c.Type);

    %% 3) Submit a small batch job that uses yaml.dump in the worker
    fprintf('Submitting test batch job...\n');

    job = batch(c, @testYamlWorkerFn, 0, ...
        {charSnakeYamlJar}, ...
        'AdditionalPaths', strsplit(path, pathsep), ...
        'CaptureDiary', true, ...
        'CurrentFolder', pwd);

    wait(job);

    fprintf('Job state: %s\n', job.State);
    fprintf('\n=========== WORKER DIARY (BEGIN) ===========\n');
    diary(job);
    fprintf('=========== WORKER DIARY (END)   ===========\n');

    delete(job);
end


function testYamlWorkerFn(charSnakeYamlJar)
% Code executed inside the batch worker.

    fprintf('Worker: ensuring SnakeYAML on Java class path...\n');
    EnsureSnakeYamlOnClasspath(charSnakeYamlJar);

    % Make sure +yaml is visible
    import yaml.*;
    % Optional: also import the Java package for debugging
    import org.yaml.snakeyaml.*;

    % Simple test payload
    S = struct();
    S.answer = 42;
    S.message = 'Hello from batch worker';
    S.time    = datestr(now);

    % Call yaml.dump
    Y = yaml.dump(S);

    fprintf('Worker: yaml.dump succeeded. YAML output:\n%s\n', Y);
end
