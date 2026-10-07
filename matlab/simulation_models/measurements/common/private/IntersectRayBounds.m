function [bHit, dEntry] = IntersectRayBounds( ...
    dOrigin, dDirection, dBoundsMin, dBoundsMax, dMinDistance, dMaxDistance, ...
    dInverseDirection, dOriginSlack) %#codegen
%% SIGNATURE
% [bHit, dEntry] = IntersectRayBounds(...)
% -------------------------------------------------------------------------------------------------------------
%% DESCRIPTION
% Intersect a ray interval with conservative axis-aligned bounds.
% Handle exactly zero direction components without multiplying zero by infinity.
% Apply the RCS-1 origin/parameter slack only to candidate bounds; triangle
% acceptance retains the exact query interval.
% -------------------------------------------------------------------------------------------------------------
%% INPUT
% dOrigin/dDirection   (3,1) ray origin and direction.
% dBoundsMin/Max       (3,1) conservative node limits.
% dMinDistance/Max     Ray-parameter interval.
% dInverseDirection    (3,1) direction reciprocals; zero for exactly zero components.
% dOriginSlack         Conservative origin-dependent padding in the mesh length unit.
% -------------------------------------------------------------------------------------------------------------
%% OUTPUT
% bHit                 Bounds overlap the ray interval.
% dEntry               Lower overlap parameter; used for traversal ordering.
% -------------------------------------------------------------------------------------------------------------
%% CHANGELOG
% 05-10-2026  Pietro Califano, Codex (GPT-6)  Add reusable prepared triangle tracing.
% 07-10-2026  Pietro Califano, Codex (GPT-6)  Adapt RCS-1 conservative bounds with cached reciprocals.
% -------------------------------------------------------------------------------------------------------------
%% DEPENDENCIES
% None.
% -------------------------------------------------------------------------------------------------------------

arguments (Input)
    dOrigin (3, 1) double
    dDirection (3, 1) double
    dBoundsMin (3, 1) double
    dBoundsMax (3, 1) double
    dMinDistance (1, 1) double
    dMaxDistance (1, 1) double
    dInverseDirection (3, 1) double
    dOriginSlack (1, 1) double
end

arguments (Output)
    bHit (1, 1) logical
    dEntry (1, 1) double
end

coder.inline('always');
dEntry = dMinDistance;
dExit = dMaxDistance;
bHit = false;

for ui8Axis = uint8(1):uint8(3)

    dLower = dBoundsMin(ui8Axis) - dOriginSlack;
    dUpper = dBoundsMax(ui8Axis) + dOriginSlack;

    if dDirection(ui8Axis) == 0
        if dOrigin(ui8Axis) < dLower || dOrigin(ui8Axis) > dUpper
            return
        end

    else

        if isfinite(dInverseDirection(ui8Axis))
            dFirst = (dLower - dOrigin(ui8Axis)) * dInverseDirection(ui8Axis);
            dSecond = (dUpper - dOrigin(ui8Axis)) * dInverseDirection(ui8Axis);
        else
            % Tiny nonzero directions can overflow their reciprocal; avoid zero*Inf.
            dFirst = (dLower - dOrigin(ui8Axis)) / dDirection(ui8Axis);
            dSecond = (dUpper - dOrigin(ui8Axis)) / dDirection(ui8Axis);
        end

        dEntry = max(dEntry, min(dFirst, dSecond));
        dExit = min(dExit, max(dFirst, dSecond));
        dParameterSlack = 64 * eps * max(1, max(abs(dEntry), abs(dExit)));

        if dEntry > dExit + dParameterSlack
            return
        end
    end
end

bHit = true;
end
