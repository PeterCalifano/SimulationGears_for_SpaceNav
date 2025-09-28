function tf = Is3dPointOn3dLine(P_test, P0_line, u_line, tol)
    if nargin<4, tol = 1e-8; end
    v = P_test - P0_line;
    % cross-product
    if norm(cross(v,u_line)) < tol
        tf = true; return;
    end
    % projection
    t = dot(v,u_line);
    Pproj = P0_line + t*u_line;
    tf = norm(P_test - Pproj) < tol;
end
