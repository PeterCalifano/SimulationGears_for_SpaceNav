function kep_mat = orbitpropagator_gauss(kep_0, t_span, mu_e, R_e, J2, B_coeff, type)
% Propagates Orbit in Keplerian Elements
% INPUT
% --> kep_0: keplerian initial parameters [km, rad]
% --> t_span: vector of times [s]
% --> mu_e: Earth gravitational parameter
% --> R_e: Earth radius [km]
% --> J2: disturbance
% --> A_M: area/mass ragio [m^2/kg]
% --> CD: drag coefficient
% --> type: 0, 1 (J2), 2 (J2 + AERO) or 3 (AERO)
%
% CONTRIBUTORS
% Tommaso Robbiani, Maria Laura Santangelo, Pietro Califano, Gennaro
% Rizzo, 01/01/2022

if size(B_coeff) == 1
    options = odeset( 'RelTol', 1e-13, 'AbsTol', 1e-14);
else
    options = odeset( 'RelTol', 1e-11, 'AbsTol', 1e-12);
end

[~, kep_mat] = ode113(@(t,y) system_gauss(t, y, mu_e, R_e, J2, B_coeff, type), t_span, kep_0, options);


end
