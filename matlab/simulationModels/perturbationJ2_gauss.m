function a_J2_RSW = perturbationJ2_gauss(a, e, i, om, th, R_e, mu_e, J2)
    % Computes J2 Acceleration in RSW frame
    %
    % INPUT
    % --> a:            Semi-major Axis                         [km]
    % --> e:            Eccentricity                            [-]
    % --> i:            Inclination                             [deg]
    % --> OM:           RAAN                                    [deg]                    
    % --> om:           Anomaly of Pericentre                   [deg] 
    % --> th:           True Anomaly                            [deg]
    % --> R_e:          Earth Radii                             [km]
    % --> mu_E:         Earth Gravitational Parameter           [km^3/s^2]
    % --> J2:           J2 Disturbance Coefficient              [-]
    %
    % OUPUT
    % --> a_J2_RSW:     J2 acceleration, RSW Coordinates        [km/s^2]
    %
    % CONTRIBUTORS
    % Tommaso Robbiani, Maria Laura Santangelo, Pietro Califano, Gennaro
    % Rizzo, 01/01/2022  


    p = a*(1-e^2); 
    r = p/(1 + e*cos(th));
    
    c = -3/2*J2*mu_e*R_e^2/r^4;
    a_r = c*(1 - 3*sin(i)^2*sin(th + om)^2);
    a_s = c*sin(i)^2*sin(2*(th+om));
    a_w = c*sin(2*i)*sin(th+om);

    a_J2_RSW = [a_r; a_s; a_w]; 
end