function [uncJ, unc_axis] = PerturbInertia(Jnom, Nsamples, Sigma)
% PROTOTYPE
% [uncJ, unc_axis] = PerturbInertia(Jnom, Nsamples, Sigma)
% -------------------------------------------------------------------------
% DESCRIPTION
% What the function does.
% -------------------------------------------------------------------------
% INPUT
%    in1    [dim]    input 1 description 
%    in2    [dim]    input 2 description
% -------------------------------------------------------------------------
% OUTPUT
%    out1    [dim]    output 1 description 
%    out2    [dim]    output 2 description
% -------------------------------------------------------------------------
% CONTRIBUTORS
%    DD-MM-AAAA    Name Surname    First Version
%    DD-MM-AAAA    Name Surname    Last Version (optional changelog)
% -------------------------------------------------------------------------
% DEPENDENCIES
%    nameOfTheFunction1InsideTheFunction.m
%    nameOfTheFunction2InsideTheFunction.m
%
% -------------------------------------------------------------------------
% Future upgrades
% Things TODO
% -------------------------------------------------------------------------
% Function beautiful wonderful colorful code here

% Get Eigenvalues and Eigenvector
% (Principal Moments of Inertia and Principal Axis)
[princ_axis, J_princ] = eig(Jnom);

% Default 3.5-Sigma value: 10% of Jnom (Gaussian distribution assumption)
if nargin < 3
    Sigma = 0.1/3.5 * J_princ * diag(randn(3, 1));
else
    if isvector(Sigma)
        Sigma = diag(Sigma);
    end
    if sum(Sigma/3 - J_princ, 'all') > 0
        error('PerturbInertia fcn exception: Sigma value too high with respect to nominal J')
    end
end

% Static allocation
% Jp_pool = zeros(3, 3, N);
uncJ = zeros(3, 3, Nsamples);
if nargout > 1
    unc_axis = zeros(3, 3, Nsamples);
end

% Initialize counter
property_check_counter = 0;
safe_var = 0;
safe_thr = 1e7;

while property_check_counter < Nsamples

    
    % Add random perturbation to eigenvalues
    Jp_perturbed = J_princ + Sigma .* diag(randn(3, 1));
    J_body_temp = princ_axis * Jp_perturbed * princ_axis';

    % Enforce symmetry of the matrix
    J_body_temp = (J_body_temp + J_body_temp')/2;

    % Get eigenpair of generated sample
    [axis_new, J_princ_new] = eig(J_body_temp);

    % Check conditions: Ixx < Iyy + Izz; Iyy < Ixx + Izz; Izz < Ixx + Iyy and
    % Positive Definite;
    if sum((diag(J_princ_new) > 0), 'all') == 3 && J_princ_new(1, 1) - J_princ_new(2, 2) - J_princ_new(3, 3) < 0 && ...
            J_princ_new(2, 2) - J_princ_new(1, 1) - J_princ_new(3, 3) < 0 && ...
            J_princ_new(3, 3) - J_princ_new(1, 1) - J_princ_new(2, 2) < 0

        % Update counter
        property_check_counter = property_check_counter + 1;
        % Assign sample to 3D array
        uncJ(:, :, property_check_counter) = J_body_temp;

        if nargout > 1
            unc_axis(:, :, property_check_counter) = axis_new;
        end
    end

    safe_var = safe_var + 1;
    if safe_var == safe_thr
        error('PerturbInertia fcn exception: while safe exit triggered')
    end

end

end