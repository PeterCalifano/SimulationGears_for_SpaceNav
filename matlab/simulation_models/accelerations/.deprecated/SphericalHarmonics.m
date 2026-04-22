function [U,DU] = SphericalHarmonics(FP, maxdeg, maxorder, mu, R0, C, S)

% [U,DU] = SphericalHarmonics(FP, maxdeg, maxorder, mu, R0, C, S)
%
% SphericalHarmonics computes the gravitational effects of a body through external Spherical Harmonics Model.
% Spherical Harmonics series expansions of the external gravitational
% potential are guaranteed to converge outside the Brillouin-sphere
% enclosing all field-generating masses.

% Author: Alessia Cremasco
% Version 1: 29/06/2023
% Version 2: 09/07/2023: changed index in the foor loop (from n = 1) and
% dimension of P from maxdeg + 3 to maxdeg + 2 
% -------------------------------------------------------------------------
%   INPUTS      unit        format          Description</strong>
% -------------------------------------------------------------------------
%   FP          [m]         (Mx3) matrix    Planet-Centered Planet-Fixed coordinates of field points.
%   maxdeg      [-]         scalar          Degree of spherical harmonics
%   maxorder    [-]         scalar          Order of spherical harmonics
%   mu          [m^3/s^2]   scalar          GM gravitational parameter of the body
%   R0          [m]         scalar          Normalizing radius
%   C           [-]         (maxdegree+1)-by-(maxdegree+1) matrix containing NORMALIZED spherical harmonic coefficients matrix, C
%   S           [-]         (maxdegree+1)-by-(maxdegree+1) matrix containing NORMALIZED spherical harmonic coefficients matrix, S
%
%   maxdeg and maxorder must be scalar integers where order <= abs(maxdeg). 
% -------------------------------------------------------------------------
%   OUTPUTS     unit       format           Description</strong>
% -------------------------------------------------------------------------
%   U           [m^2/s^2]  (Mx1) vector    Gravitational potential
%   DU          [m/s^2]    (Mx3) matrix    Gradient of potential
%                                           (acceleration) in the Planet-Centered Planet-Fixed reference frame
% -------------------------------------------------------------------------
%   References:  
%   [1] Vallado, D. A., "Fundamentals of Astrodynamics and Applications",
%       McGraw-Hill, New York, 1997.  
%   [2] Gottlieb, R. G., "Fast Gravity, Gravity Partials, Normalized Gravity, 
%       Gravity Gradient Torque and Magnetic Field: Derivation, Code and Data," 
%       Technical Report NASA Contractor Report 188243, NASA Lyndon B. Johnson 
%       Space Center, Houston, TX, February 1993.

% Check inputs
if maxorder > maxdeg
    error('order must be <= degree maxdeg')
end
if (size( FP, 2) ~= 3)
    error(message('Wrong Dimension of field points vector'));
end

upperlimit = size(C,1); 
if maxdeg > upperlimit
    error(message('Exceed MaxDeg'));
end

% Compute geocentric radius
r = sqrt( sum( FP.^2, 2 ));
% Check if geocentric radius is less than reference radius
if any(r < R0)
   warning('Geocentric radius of field point less than reference radius');
end

% Compute geocentric latitude
delta = asin( FP(:,3)./ r );

% Compute longitude                                                           
lambda = atan2( FP(:,2), FP(:,1) );

% inizialization
smlambda = zeros( size(FP,1), maxorder + 1 ); % sin(m*lambda)
cmlambda = zeros( size(FP,1), maxorder + 1 ); % cos(m*lambda)

slambda = sin(lambda);
clambda = cos(lambda);
smlambda(:,1) = 0;          % m = 0
cmlambda(:,1) = 1;          % m = 0
smlambda(:,2) = slambda;    % m = 1    
cmlambda(:,2) = clambda;    % m = 1

% recursive formula for sin(m*lambda) and cos(m*lambda)
for order=3:maxorder+1
    % order is m
    smlambda(:,order) = 2.0 .* clambda .* smlambda(:, order-1) - smlambda(:, order-2); 
    cmlambda(:,order) = 2.0 .* clambda .* cmlambda(:, order-1) - cmlambda(:, order-2);
end

% Compute normalized associated Legendre polynomials
[P,scaleFactor] = LegendrePolynomials( delta, maxdeg, maxorder );

% Compute gravitational potential and acceleration in PCEF coordinates
[U,gx,gy,gz] = SphericalHarmonicsGravity( FP, maxdeg, maxorder, P, C( 1:maxdeg+1, 1:maxdeg+1 ), ...
                                  S( 1:maxdeg+1, 1:maxdeg+1 ), smlambda, ...
                                  cmlambda, mu, R0, r,scaleFactor );

% Acceleration vector in PCEF coordinates
DU = [gx gy gz];

%% LOCAL FUNCTIONS
function [P,scaleFactor] = LegendrePolynomials( delta, maxdeg, maxorder )
% LegendrePolynomials local function computing normalized associated Legendre polynomials, P, via recursion relations for spherical harmonic gravity 
% [P,scaleFactor] = LegendrePolynomials( delta, maxdeg, maxorder )
% -------------------------------------------------------------------------
%   INPUTS      unit        format          Description</strong>
% -------------------------------------------------------------------------
%   delta       [rad]       (Mx1) vector    Geocentric latitude
%   maxdeg      [-]         scalar          Degree of spherical harmonics
%   maxorder    [-]         scalar          Order of spherical harmonics
% -------------------------------------------------------------------------
%   OUTPUTS     unit       format           Description</strong>
% -------------------------------------------------------------------------
%   P           [-]        (maxdeg+2,maxorder+2,M) matrix          Normalized associated Legendre polynomials
%   scaleFactor [-]        (maxdeg+2,maxorder+2,M) matrix          Scale Factor
% -------------------------------------------------------------------------

% initialization
P = zeros(maxdeg+2, maxorder+2, length(delta));
scaleFactor = zeros(maxdeg+2, maxorder+2, length(delta));

cphi = cos(pi/2-delta); % sin(delta)
sphi = sin(pi/2-delta); % cos(delta)

% force numerically zero values to be exactly zero
cphi(abs(cphi)<=eps) = 0;
sphi(abs(sphi)<=eps) = 0;
 
% Seeds for recursion formula
P(1,1,:) = 1;            % n = 0, m = 0;
P(2,1,:) = sqrt(3)*cphi; % n = 1, m = 0;  
scaleFactor(1,1,:) = 0;  % n = 0, m = 0;
scaleFactor(2,1,:) = 1;  % n = 1, m = 0; 
P(2,2,:) = sqrt(3)*sphi; % n = 1, m = 1;
scaleFactor(2,2,:) = 0;  % n = 1, m = 1;

% RECURSIVE RELATIONS:
% vertical : (n-m)P_{n,m}sin(delta) = (2n-1)sin(delta)P_{n-1,m} - (n+m-1)P_{n-2,m}sin(delta)
% sectorial (diagonal) : P_{n,n} = (2n-1)cos(delta)P_{n-1,n-1}
% NORMALIZATION
% 4π normalized: N_{n,m} = sqrt(2- 𝛿_{0,m}(2n+1)(n-m)!/(n+m)!)

for n = 2:maxdeg+1 
    k = n + 1; % index for degree
    for m = 0:min(n, maxorder+1)
        p = m + 1; % index for order
        % Compute normalized associated Legendre polynomials, P, via recursion relations 
        % Scale Factor needed for normalization of dUddelta partial derivative            
        if (n == m)           
            P(k,k,:) = sqrt(2*n+1) / sqrt(2*n) * sphi .* reshape(P(k-1,k-1,:),size(delta));
            scaleFactor(k,k,:) = 0;
        elseif (m == 0)
            P(k,p,:) = (sqrt(2*n+1) / n) * (sqrt(2*n - 1) * cphi .* reshape(P(k-1,p,:),size(delta)) - (n-1) / sqrt(2*n - 3) * reshape(P(k-2,p,:),size(delta)));
            scaleFactor(k,p,:) = sqrt( (n+1)*(n)/2);
        else
            P(k,p,:) = sqrt(2*n+1) / (sqrt(n+m)*sqrt(n-m)) * (sqrt(2*n-1) * cphi .* reshape(P(k-1,p,:),size(delta)) - sqrt(n + m - 1) * sqrt(n-m-1) / sqrt(2*n - 3) * reshape(P(k-2,p,:),size(delta)));
            scaleFactor(k,p,:) = sqrt( (n+m+1)*(n-m));
        end
    end
end
end

%=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
function [U,gx,gy,gz] = SphericalHarmonicsGravity(p,maxdeg,maxorder,P,C,S,smlambda,cmlambda,GM,Re,r,scaleFactor)
% SphericalHarmonicsGravity internal function computing gravity in planet-centered
% planet-fixed (PCEF) coordinates using PCPF position, desired
% degree/order, normalized associated Legendre polynomials, normalized
% spherical harmonic coefficients, trigonometric functions of longitude, planetary constant, and radius to center of
% planet.
% [U,gx,gy,gz] = SphericalHarmonicsGravity(p,maxdeg,maxorder,P,C,S,smlambda,cmlambda,GM,Re,r,scaleFactor)
% -------------------------------------------------------------------------
%   INPUTS      unit        format          Description</strong>
% -------------------------------------------------------------------------
%   p           [m]         (Mx3) matrix    Planet-Centered Planet-Fixed coordinates of field points.
%   maxdeg      [-]         scalar          Degree of spherical harmonics
%   maxorder    [-]         scalar          Order of spherical harmonics
%   P           [-]         (maxdeg+2,maxorder+2,M) matrix          Normalized associated Legendre polynomials
%   C           [-]         (degree+1)-by-(degree+1) matrix containing normalized spherical harmonic coefficients matrix, C
%   S           [-]         (degree+1)-by-(degree+1) matrix containing normalized spherical harmonic coefficients matrix, S
%   smlambda    [-]         (M x maxorder+1) matrix containing sin(m*lambda)
%   cmlambda    [-]         (M x maxorder+1) matrix containing cos(m*lambda)
%   GM          [m^3/s^2]   scalar          GM gravitational parameter of the body
%   Re          [m]         scalar          Normalizing radius
%   r           [m]         (Mx1) vector    Geocentric radius
%   scaleFactor [-]         (maxdeg+2,maxorder+2,M) matrix          Scale Factor
% -------------------------------------------------------------------------
%   OUTPUTS     unit       format           Description</strong>
% -------------------------------------------------------------------------
%   U           [m^2/s^2]  (Mx1) vector    Gravitational potential
%   gx          [m/s^2]    (Mx1) vector    Gravitational acceleration in the Planet-Centered Planet-Fixed reference frame x-direction
%   gy          [m/s^2]    (Mx1) vector    Gravitational acceleration in the Planet-Centered Planet-Fixed reference frame y-direction
%   gz          [m/s^2]    (Mx1) vector    Gravitational acceleration in the Planet-Centered Planet-Fixed reference frame z-direction
% -------------------------------------------------------------------------

rRatio   = Re./r;
rRatio_n = 1;

% Initialize summation degree-wise of gravity in radial coordinates (n = 0)
dUdrSumN      = 1;
dUdphiSumN    = 0;
dUdlambdaSumN = 0;

U = GM./r; 

% Summation of gravity in radial coordinates
for n = 1:maxdeg 
    k = n+1; % index for degree   
    rRatio_n      = rRatio_n.*rRatio;
    % Initialize summation order-wise
    dUdrSumM      = 0;
    dUdphiSumM    = 0;
    dUdlambdaSumM = 0;
    for m = 0:min(n,maxorder)
        j = m+1; % index for order   
        % order-wise summation
        dUdrSumM      = dUdrSumM + reshape(P(k,j,:),size(r)) .* (C(k,j) .* cmlambda(:,j) + S(k,j) .* smlambda(:,j)); 
        dUdphiSumM    = dUdphiSumM + ( (reshape(P(k,j+1,:),size(r)) .* reshape(scaleFactor(k,j,:),size(r))) - p(:,3) ./ (sqrt(p(:,1).^2 + p(:,2).^2)) .* m .* reshape(P(k,j,:),size(r))) .* (C(k,j) .* cmlambda(:,j) + S(k,j) .* smlambda(:,j)); 
        dUdlambdaSumM = dUdlambdaSumM + m * reshape(P(k,j,:), size(r)) .* (S(k,j) .* cmlambda(:,j) - C(k,j) .* smlambda(:,j));
        U = U + GM * (Re ./ r) .^(n + 1) / Re .* reshape(P(k,j,:),size(r)) .* (C(k,j) .* cmlambda(:,j) + S(k,j) .* smlambda(:,j)); 
    end
    % degree-wise summation
    dUdrSumN      = dUdrSumN      + dUdrSumM .* rRatio_n .* k;
    dUdphiSumN    = dUdphiSumN    + dUdphiSumM .* rRatio_n;
    dUdlambdaSumN = dUdlambdaSumN + dUdlambdaSumM .* rRatio_n;
end

% gravity in spherical coordinates
dUdr      = -GM ./ (r .* r) .* dUdrSumN;
dUdphi    =  GM ./ r .* dUdphiSumN;
dUdlambda =  GM ./ r .* dUdlambdaSumN;

% gravity in PCPF coordinates
gx = ((1 ./ r) .* dUdr - (p(:,3) ./ (r .* r .* sqrt(p(:,1).^2 + p(:,2).^2))) .* dUdphi) .* p(:,1) ...
      - (dUdlambda ./ (p(:,1).^2 + p(:,2).^2)) .* p(:,2); 
gy = ((1 ./ r) .* dUdr - (p(:,3) ./ (r .* r .* sqrt(p(:,1).^2 + p(:,2).^2))) .* dUdphi) .* p(:,2) ...
      + (dUdlambda ./ (p(:,1).^2 + p(:,2).^2)) .* p(:,1); 
gz = (1 ./ r) .* dUdr .* p(:,3) + ((sqrt(p(:,1).^2 + p(:,2).^2)) ./ (r .* r)) .* dUdphi;

% Special case for poles
atPole = abs(atan2(p(:,3), sqrt(p(:,1).^2 + p(:,2).^2)))==pi/2;
if any(atPole)
    gx(atPole) = 0;
    gy(atPole) = 0;
    gz(atPole) = (1 ./ r(atPole)) .* dUdr(atPole) .* p((atPole),3);
end

end


end
