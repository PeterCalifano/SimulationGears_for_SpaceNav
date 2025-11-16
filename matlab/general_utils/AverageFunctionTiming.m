function [dAvgRunTime, dTimings] = AverageFunctionTiming(fcnHandle, ui32NumTrials)
arguments
    fcnHandle     (1,1)  {isa(fcnHandle, 'function_handle')}
    ui32NumTrials (1,1) uint32 = 100
end

dTimings = nan(ui32NumTrials, 1);

for idT = 1:ui32NumTrials
    fprintf("\rRunning trial %d of %d", idT, ui32NumTrials)
    pause(0.00001);
    tic
    fcnHandle();
    dTimings(idT) = toc;
end

dAvgRunTime = mean(dTimings, 'omitnan');
fprintf("\nAveraged execution time %6.4g ms\n", 1000 * dAvgRunTime)
end
