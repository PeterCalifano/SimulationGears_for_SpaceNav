function dTimegridOut = ConvertETfromTimeFormat(dTimegridIn, charInputTimeRef)%#codegen
arguments
    dTimegridIn              double   {mustBeReal, mustBeFinite}
    charInputTimeRef (1,:)   char     {coder.mustBeConst, mustBeMember(charInputTimeRef, ["ET","GPS","TYVAK_ET", "TYVAK_GPS"])}
end
% BUG: conversion does not seem correct
% Reference epoch in GPS seconds, precomputed as a compile-time constant
% dReferenceEpochGPS_GPS = coder.const(cspice_unitim( cspice_str2et('06 JAN 1980 00:00:00 UTC'), 'ET','GPS'));

dReferenceEpochGPS_ET  = coder.const(-630763148.815937);
dReferenceEpochGPS_GPS = coder.const(-630763200);

bIsAvailableSPICE = coder.const(coder.target('MATLAB') && not(isempty(which("mice"))));

switch upper(charInputTimeRef)

    case 'ET'

        % No conversion needed
        dTimegridOut = dTimegridIn;

    case 'GPS'

        if bIsAvailableSPICE
            % Convert from GPS time to ET
            dTimegridOut = cspice_unitim(dTimegridIn, 'GPS', 'ET');
        else
            warning('Conversion GPS to ET through SPICE not supported when codegen.')
        end

    case 'TYVAK_ET'
        % Add fixed TYVAK reference (in ET)
        dTimegridOut  = dTimegridIn + dReferenceEpochGPS_ET;

    case 'TYVAK_GPS'
        % Add fixed TYVAK reference (in GPS seconds)
        dTimegridOut  = cspice_unitim( dTimegridIn + dReferenceEpochGPS_GPS, 'GPS', 'ET');

    otherwise
        % Should never happen because of mustBeMember, but kept for robustness
        assert(0, sprintf('ConvertETfromTimeFormat:InvalidInputTimeRef. Unsupported input time reference "%s".', charInputTimeRef));
end

end
