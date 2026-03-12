function EnsureSnakeYamlOnClasspath(charSnakeYamlJar, bRunTest)
arguments
    charSnakeYamlJar    (1,:) char {mustBeFile}
    bRunTest            (1,1) logical = false
end
% Add the SnakeYAML jar to the dynamic Java class path in this process.

    if exist(charSnakeYamlJar, 'file') ~= 2
        error('Worker: SnakeYAML JAR not found at %s', charSnakeYamlJar);
    end

    dynCP = javaclasspath('-dynamic');
    if ~any(strcmp(dynCP, charSnakeYamlJar))
        javaaddpath(charSnakeYamlJar);
        fprintf('Worker: javaaddpath(%s)\n', charSnakeYamlJar);
    else
        fprintf('Worker: JAR already on Java path.\n');
    end

    % Touch the class so that failures are immediate and explicit
    import org.yaml.snakeyaml.*;
    if bRunTest
        try
            DumperOptions();
            fprintf('Worker: DumperOptions is available.\n');
        catch ME
            error('Worker: DumperOptions not available after javaaddpath: %s', ME.message);
        end
    end
end
