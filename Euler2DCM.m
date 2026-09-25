function out = Euler2DCM(phi,theta,psi)
% Matriz cambio de base Euler a Direction Cosine Matrix
ctheta = cos(theta);
stheta = sin(theta);
sphi = sin(phi);
cphi = cos(phi);
spsi = sin(psi);
cpsi = cos(psi);

out = [ctheta*cpsi,  sphi*stheta*cpsi-cphi*spsi,  cphi*stheta*cpsi+sphi*spsi;
       ctheta*spsi,  sphi*stheta*spsi+cphi*cpsi,  cphi*stheta*spsi-sphi*cpsi;
         -stheta,            sphi*ctheta,               cphi*ctheta];