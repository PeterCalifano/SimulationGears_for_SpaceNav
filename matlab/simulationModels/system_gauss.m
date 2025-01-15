function fn = system_gauss(t, y, mu_e, R_e, J2, B_coeff, type)
    % Builds ODE system in Keplerian Parameter
    %
    % INPUT
    % fn            ode system                          [-]
    %
    % OUTPUT
    % t:            time                                [s]
    % y:            state                               [km, km/s]
    % mu_e:         Earth Gravitational Parameter       [km^3/s^2]
    % R_e:          Earth Radius                        [km]
    % J2:           J2 Disturbance Coefficient          [-]
    % B_coeff:      B vector coefficient                [m^2/kg]
    % type:         Propagation Type                    [-]
    %
    % CONTRIBUTORS
    % Tommaso Robbiani, Maria Laura Santangelo, Pietro Califano, Gennaro
    % Rizzo, 01/01/2022




% type = 0 -> unperturbed
% type = 1 -> J2
% type = 2 -> J2 + Aero
fn = zeros(size(y));
a = y(1);
e = y(2);
i = y(3);
OM = y(4);
om = y(5);
th = y(6);

% Constants
p = a*(1-e^2);
r = p/(1 + e*cos(th));
b = a*sqrt(1-e^2);
n = sqrt(mu_e/a^3);
h = n*a*b;

B_value = polyval(B_coeff, t); 

% fprintf("\nB = %f, \nT = %f", B_value, t);

%DRAG
a_drag_RSW = aerodrag_gauss(a, e, i, OM, om, th, mu_e, B_value, R_e);

%J2
a_J2_RSW = perturbationJ2_gauss(a, e, i, om, th, R_e, mu_e, J2);


if type == 0
    a_r = 0;
    a_s = 0;
    a_w = 0;
elseif type == 1
    a_r = a_J2_RSW(1);
    a_s = a_J2_RSW(2);
    a_w = a_J2_RSW(3);
elseif type == 2
    a_r = a_J2_RSW(1) + a_drag_RSW(1);
    a_s = a_J2_RSW(2) + a_drag_RSW(2);
    a_w = a_J2_RSW(3) + a_drag_RSW(3);
elseif type == 3
    a_r = a_drag_RSW(1);
    a_s = a_drag_RSW(2);
    a_w = a_drag_RSW(3);
end


fn(1) = 2*a^2/h * (e*sin(th)*a_r + p/r*a_s);
fn(2) = 1/h*(p*sin(th)*a_r + ((p + r)*cos(th) + r*e)*a_s);
fn(3) = r*cos(th + om)*a_w/h;
fn(4) = r*sin(th+om)*a_w/(h*sin(i));
fn(5) = 1/(h*e)*(-p*cos(th)*a_r + (p + r)*sin(th)*a_s) - (r*sin(th+om)*cos(i)*a_w)/(h*sin(i));
fn(6) = h/r^2 + 1/(e*h)*(p*cos(th)*a_r - (p+r)*sin(th)*a_s);

end