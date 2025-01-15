function [outputArg1,outputArg2] = RHS_GaussOE(a_pertubing_RSW, kep, mu, unit)
%% PROTOTYPE
% -------------------------------------------------------------------------
%% DESCRIPTION
% -------------------------------------------------------------------------
%% INPUT
% kep = [a, i, e, RAAN, omega, f]
% -------------------------------------------------------------------------
%% OUTPUT
% -------------------------------------------------------------------------
%% CONTRIBUTORS
% Pietro Califano
% ------------------------------------------------------------------------
%% CHANGELOG
% V1: 
% -------------------------------------------------------------------------
%% Next upgrades
%

%% Function code
a = kep(1);
incl = kep(2);
e = kep(3);
RAAN = kep(4);
arg_peri = kep(5);
f = kep(6);

if nargin <= 3
    unit = 'deg';
    disp('Default unit: rad --> deg2rad conversion occurred')
end

if unit == 'deg'
    incl = deg2rad(incl);
    arg_peri = deg2rad(arg_peri);
    RAAN = deg2rad(RAAN);
    f = deg2rad(f);
end


% Compute required quantities from kep
p = a*(1-e^2);
h = sqrt(p*mu);
r = (h^2/mu) * (1/(1+e*cos(f)));

% Perturbing acceleration components
a_r = a_pertubing_RSW(1);
a_s = a_pertubing_RSW(2);
a_w = a_pertubing_RSW(3);

acc_norm = norm(a_pertubing_RSW); % norm of the acceleration

% System of equation to integrate
Da = 2 * (acc_norm^2/h) * (e*sin(f)*a_r + p*a_s/r);
De = (1/h)*(p*sin(f)*a_r + ((p+r)*cos(f) + r*e)*a_s);
Di = (r*cos(f + arg_peri))*a_w/h;
DRAAN = (r*cos(f + arg_peri))*a_w./(h*sin(incl));
Darg_peri = (1/(h*e)) * (-p*cos(f)*a_r + (p+r)*sin(f)*a_s) - (r*cos(f + arg_peri))*cos(incl)*a_w./(h*sin(incl));
Df = (h/r^2) + (1/(h*e)) * (p*cos(f)*a_r - (p+r)*sin(f)*a_s);
























end