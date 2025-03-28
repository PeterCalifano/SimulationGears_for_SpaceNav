function [ddt_Jac_A,ddt_Jac_B,dMdtA,dMdtB,GaussEQ] = getNonlinearHyperBolEquations()
% returns the d/dt of orbital elements in hyperbolic trajectory, and d/df
% of the same 
a = casadi.MX.sym('a',1,1); 
e = casadi.MX.sym('e',1,1);
w = casadi.MX.sym('w',1,1);
i = casadi.MX.sym('i',1,1);
O = casadi.MX.sym('O',1,1);
M = casadi.MX.sym('M',1,1);

xlin =  casadi.MX.sym('xlin',6,1);
ulin =  casadi.MX.sym('ulin',3,1);

f = casadi.MX.sym('f',1,1);
Fx = casadi.MX.sym('Fx',1,1);
Fy = casadi.MX.sym('Fy',1,1);
Fz = casadi.MX.sym('Fz',1,1);
mu = casadi.MX.sym('mu',1,1);




n = sqrt(mu/abs(a)^3);
r =  a*(e^2-1)/(1 + e*cos(f));
nu = sqrt(e^2-1);
Pu = Fx;
Pv = Fy;
Pn = Fz;

%Pv = -Fx;

GaussEQ = [-2*e*sin(f)*Pu /(n*nu) - 2*a*nu*Pv/(n*r);
  nu*sin(f)*Pu/(a*n) + (nu^3/(e*r*n) + r*nu/(a^2*e*n) )*Pv;
  r*cos(w+f)*Pn/(a^2*n*nu);
  r*sin(w+f)*Pn/(a^2*n*sin(i)*nu);
  -nu*cos(f)*Pu/(a*e*n) + r*(2+e*cos(f))*sin(f)*Pv/(a^2*n*e*nu) - r*sin(w+f)*cos(i)*Pn/(a^2*n*sin(i)*nu);
  n + (2*r/(a^2*n) - nu^2*cos(f)/(a*e*n))*Pu + r*(2+e*cos(f))*sin(f)*Pv/(a^2*e*n)];

dMdtA = [n];
dMdtB = (2*r/(a^2*n) - nu^2*cos(f)/(a*e*n))*Pu + r*(2+e*cos(f))*sin(f)*Pv/(a^2*e*n);


% n in last term removed
ddt = [-2*e*sin(f)*Pu/(n*nu) - 2*a*nu*Pv/(n*r);
  nu*sin(f)*Pu/(a*n) + (nu^3/(e*r*n) + r*nu/(a^2*e*n) )*Pv;
  r*cos(w+f)*Pn/(a^2*n*nu);
  r*sin(w+f)*Pn/(a^2*n*sin(i)*nu);
  nu*cos(f)*Pu/(a*e*n) - r*(2+e*cos(f))*sin(f)*Pv/(a^2*n*e*nu) - r*sin(w+f)*cos(i)*Pn/(a^2*n*sin(i)*nu);
   n + ( 2*r/(a^2*n) + nu^2*cos(f)/(a*e*n))*Pu - r*(2+e*cos(f))*sin(f)*Pv/(a^2*e*n)];


%
% dt = [+2*e*sin(f)*Pu /(n*nu) + 2*a*nu*Pv/(n*r);
%   -nu*sin(f)*Pu/(a*n) - (nu^3/(e*r*n) + r*nu/(a^2*e*n) )*Pv;
%   -r*cos(w+f)*Pn/(a^2*n*nu);
%   -r*sin(w+f)*Pn/(a^2*n*sin(i)*nu);
%   nu*cos(f)*Pu/(a*e*n) - r*(2+e*cos(f))*sin(f)*Pv/(a^2*n*e*nu) + r*sin(w+f)*cos(i)*Pn/(a^2*n*sin(i)*nu);
%    ( -2*r/(a^2*n) - nu^2*cos(f)/(a*e*n))*Pu - r*(2+e*cos(f))*sin(f)*Pv/(a^2*e*n)];


ddt_Jac_A = ddt.jacobian([a,e,i,O,w,M]);
ddt_Jac_B = ddt.jacobian([Fx,Fy,Fz]);


u = [Fx,Fy,Fz];
ddt_Jac_A = casadi.Function('ddt_Jac_A',{[a,e,i,O,w,M],f,u,mu},{ddt_Jac_A});
ddt_Jac_B = casadi.Function('ddt_Jac_B',{[a,e,i,O,w,M],f,u,mu},{ddt_Jac_B});

dMdtA = casadi.Function('ddt_Jac_A',{[a,e,i,O,w,M],f,u,mu},{dMdtA});
dMdtB = casadi.Function('ddt_Jac_B',{[a,e,i,O,w,M],f,u,mu},{dMdtB});

GaussEQ = casadi.Function('GaussEQ',{[a,e,i,O,w,M],f,u,mu},{GaussEQ})

%lin =  casadi.Function('Lin',{[a,e,i,O,w,M],xlin,ulin,f,Fx,Fy,Fz,mu},{A_lin + B_lin});
%f1 = casadi.Function('ddtA',{[aneg,e,i,O,w,f],u,mu},{ddtFullA});
%f2 = casadi.Function('ddtB',{[aneg,e,i,O,w,f],u,mu},{ddtFullB});

%f5 = casadi.Function('ddtFUll',{[aneg,e,i,O,w,f],u,mu},{ddTfull});
