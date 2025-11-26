function dTimegridOut = ConvertTimeFormatFromET(dTimegridIn, charTargetTimeRef)%#codegen
arguments
    dTimegridIn               double   {mustBeReal, mustBeFinite}
    charTargetTimeRef (1,:)   char     {coder.mustBeConst, mustBeMember(charTargetTimeRef, ["ET","GPS","TYVAK_ET", "TYVAK_GPS"])}
end

% TYVAK reference epoch in GPS seconds, precomputed as a compile-time constant
% dTyvakGpsReference = coder.const(cspice_unitim( cspice_str2et('06 JAN 1980 00:00:00 UTC'), ...  % TYVAK reference in ET
%                                                               'ET', ...
%                                                               'GPS'));
dTyvakReference_ET  = coder.const(-630763148.815937);
dTyvakReference_GPS = coder.const(-630763200);

bIsAvailableSPICE = coder.const(coder.target('MATLAB') && not(isempty(which("mice"))));

switch upper(charTargetTimeRef)

    case 'ET'

        % No conversion needed
        dTimegridOut = dTimegridIn;

    case 'GPS'

        if bIsAvailableSPICE
            % Convert from ET to GPS time
            dTimegridOut = cspice_unitim(dTimegridIn, 'ET', 'GPS');
        else
            warning('Conversion ET to GPS through SPICE not supported when codegen.')
        end

    case 'TYVAK_ET'
        % Subtract fixed TYVAK reference (in ET)
        dTimegridOut  = dTimegridIn - dTyvakReference_ET;

    case 'TYVAK_GPS'
        % Convert ET -> GPS, then subtract fixed TYVAK reference (in GPS seconds)
        dTimegridOut  = dTimegridIn - dTyvakReference_GPS;

    otherwise
        % Should never happen because of mustBeMember, but kept for robustness
        assert(0, sprintf('ConvertTimeFormatFromET:InvalidTargetTimeRef. Unsupported target time reference "%s".', charTargetTimeRef));
end

end
