function R = Axis2DCM(nhat)
% Input: nhat -> 3x1 axis direction vector
% Output: R -> 3x3 Direction Cosine Matrix
x = nhat(1);
y = nhat(2);
z = nhat(3);

%%% yaw 
psi = atan2(y,x);
%%% pitch
theta = atan2(z,sqrt(x^2+y^2));
%%% Phi siempre es 0
phi = 0;

R = Euler2DCM(phi,theta,psi);