function [dxdt] = nbody_rhs(t, x, bodies, frame)
%NBODY_RHS Evaluates the right-hand-side of a N-body propagator
%   Evaluates the right-hand-side of a newtonian N-body propagator.
%   The integration centre is the Solar-System-Barycentre (SSB) and only
%   Newtonian gravitational accelerations are considered.
%
%
% Author
%   Name: ALESSANDRO 
%   Surname: MORSELLI
%   Research group: DART
%   Department: DAER
%   University: Politecnico di Milano 
%   Creation: 26/09/2021
%   Contact: alessandro.morselli@polimi.it
%   Copyright: (c) 2021 A. Morselli, Politecnico di Milano. 
%                  All rights reserved.
%
%
% Notes:
%   This material was prepared to support the course 'Satellite Guidance
%   and Navigation', AY 2021/2022.
%
%
% Inputs:
%   t      : [1,1] ephemeris time (ET SPICE), seconds past J2000 (TDB)
%   x      : [6,1] cartesian state vector wrt Solar-System-Barycentre
%   bodies : [1,6] cell-array created with function nbody_init
%
% Outputs:
%   dxdt   : [6,1] RHS, newtonian gravitational acceleration only
%
% Prerequisites:
%   - MICE (Matlab SPICE)
%   - Populated kernel pool (SPK, LSK, PCK kernels)
%

if not( strcmpi(frame, 'ECLIPJ2000') || strcmpi(frame, 'J2000') )
    msg = 'Invalid integration reference frame, select either J2000 or ECLIPJ2000';
    error(msg);
end

% Initialize right-hand-side
dxdt = zeros(6,1);

% Position detivative is object's velocity
dxdt(1:3) = x(4:6);

% Extract the object position from state x
rr_ssb_obj = x(1:3);

for i=1:length(bodies)

    % Retrieve position and velocity of i-th celestial body wrt Solar
    % System Barycentre in inertial frame
    rv_ssb_body = cspice_spkezr(bodies{i}.name, t, frame, 'NONE', 'SSB');

    % Extract object position wrt. i-th celestial body
    rr_body_obj = rr_ssb_obj - rv_ssb_body(1:3);

    % Compute square distance and distance
    dist2 = dot(rr_body_obj, rr_body_obj);
    dist = sqrt(dist2);

    % Compute the gravitational acceleration using Newton's law
    aa_grav =  - bodies{i}.GM * rr_body_obj /(dist*dist2);

    % Sum up acceleration to right-hand-side
    dxdt(4:6) = dxdt(4:6) + aa_grav;

end

end

