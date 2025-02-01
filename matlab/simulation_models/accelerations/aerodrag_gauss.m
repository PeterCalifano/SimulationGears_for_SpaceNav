function a_drag_RSW = aerodrag_gauss(a, e, i, OM, om, th, mu_e, B, R_e)
    
    % Computes Drag in RSW frame
    %
    % INPUT
    % --> a:            Semi-major Axis                         [km]
    % --> e:            Eccentricity                            [-]
    % --> i:            Inclination                             [deg]
    % --> OM:           RAAN                                    [deg]                    
    % --> om:           Anomaly of Pericentre                   [deg] 
    % --> th:           True Anomaly                            [deg]
    % --> mu_e:         Earth Gravitational Parameter           [km^3/s^2]
    % --> B:            B coefficient, A/M*CD                   [m^2/kg]
    % --> R_e:          Earth Radii                             [km]
    %
    % OUPUT
    % --> a_drag_RSW:   Drag acceleration, RSW Coordinates      [km/s^2]
    %
    % CONTRIBUTORS
    % Tommaso Robbiani, Maria Laura Santangelo, Pietro Califano, Gennaro
    % Rizzo, 01/01/2022   


    [rr, vv] = kep2car(a, e, i, OM, om, th, mu_e);
    om_e = [0; 0; 7.292*1e-5]; %Earth Angular Velocity [rad/s] 
    v_rel = vv - cross(om_e, rr); %Relative Velocity in ECEI, km/s
    v_rel = v_rel*1e3; %m/s
    height = norm(rr) - R_e; % km

    if height > 70 && height < 80
        h_0 = 70; 
        rho_0 = 8.770*1e-5; 
        H = 6.549;
    elseif height > 80 && height < 90
        h_0 = 80; 
        rho_0 = 1.905*1e-5; 
        H = 5.799;
    elseif height > 90 && height < 100
        h_0 = 90; 
        rho_0 = 3.396*1e-6; 
        H = 5.382;
    elseif height > 100 && height < 110
        h_0 = 100; 
        rho_0 = 5.297*1e-7; 
        H = 5.877;
    elseif height > 110 && height < 120
        h_0 = 110; 
        rho_0 = 9.661*1e-8; 
        H = 7.263;
    elseif height > 120 && height < 130
        h_0 = 120; 
        rho_0 = 2.438*1e-8; 
        H = 9.473; 
    elseif height > 130 && height < 140
        h_0 = 130; 
        rho_0 = 8.484*1e-9; 
        H = 12.636; 
    elseif height > 140 && height < 150
        h_0 = 140; 
        rho_0 = 3.845*1e-9; 
        H = 16.149; 
    elseif height > 150 && height < 180
        h_0 = 150; 
        rho_0 = 2.070*1e-9; 
        H = 22.523; 
    elseif height > 180 && height < 200
        h_0 = 180; 
        rho_0 = 5.464*1e-10; 
        H = 29.740;  
    elseif height > 200 && height < 250
        h_0 = 200; 
        rho_0 = 2.789*1e-10; 
        H = 37.105; 
    elseif height > 250 && height < 300
        h_0 = 250; 
        rho_0 = 7.248*1e-11;  
        H = 45.546;  
    elseif height > 300 && height <= 350
        h_0 = 300; 
        rho_0 = 2.418*1e-11; 
        H =53.628; 
    elseif height > 350 && height <= 400
        h_0 = 350; 
        rho_0 = 9.158*1e-12; 
        H = 53.298; 
   elseif height > 400 && height <= 450
        h_0 = 400; 
        rho_0 = 3.725*1e-12; 
        H = 58.515;
   elseif height > 450 && height <= 500
        h_0 = 450; 
        rho_0 = 1.585*1e-12; 
        H = 60.828;
    elseif height > 500 && height <= 600
        h_0 = 500; 
        rho_0 = 6.967*1e-13; 
        H = 63.922; 
    elseif height > 600 && height <= 700
        h_0 = 600; 
        rho_0 = 1.454*1e-13; 
        H = 71.835;
    elseif height > 700 && height <= 800
        h_0 = 700; 
        rho_0 = 3.614*1e-14; 
        H = 88.667; 
    elseif height > 800 && height <= 900
        h_0 = 800; 
        rho_0 = 1.170*1e-14; 
        H = 124.64;
    elseif height > 900 && height <= 1000
        h_0 = 900; 
        rho_0 = 5.245*1e-15;  
        H = 181.05;
    elseif height > 1000
        h_0 = 1000; 
        rho_0 = 3.019*1e-15; 
        H = 268; 
    end

    rho = rho_0*exp(-(height-h_0)/H); 
    a_drag_ECEI = -0.5*B*rho*norm(v_rel)*v_rel; %ECEI, m/s^2
   
    %Conversion of a_drag from ECEI -> RSW 
    l1 = rr/norm(rr);

    hh = cross(rr,vv);
    l3 = hh/norm(hh);

    l2 = cross(l3,l1);

    ALN = [l1, l2, l3]';
    a_drag_RSW = ALN*a_drag_ECEI; 
    a_drag_RSW = a_drag_RSW*1e-3; %km/s^2


end