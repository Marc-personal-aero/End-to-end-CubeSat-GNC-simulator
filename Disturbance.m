function [XYZD,LMND] = Disturbance(altitude,Amax,lmax,vel,CD,BI_Tesla)
%%% Fuerzas y Momentos externos que afectan al satélite
global rwSATURATED

%%% Resistencia aerodinámica
V = norm(vel);
d = density(altitude);
Fdrag = (1/2)*d*V^2*Amax*CD;
vhat = vel / V;
XYZAERO = -Fdrag*vhat; %%Vector fuerza aerodinámica 
Maero = Fdrag*lmax/2;  %%Momento aerodinámico
LMNAERO = [Maero;Maero;Maero];

%%% Presión por radiación solar
solar_pressure = 4.5e-6; %%Pa
Fpressure = solar_pressure*Amax;
shat = vhat;  %%Se debería de codificar la dirección del sol
XYZSRP = -Fpressure*shat;  %%Vector de la fuerza ejercida por la radiación
Mpressure = Fpressure*lmax/2;  %%Momento de la radiación solar
LMNSRP = [Mpressure;Mpressure;Mpressure];

%%% Momento por dipolo magnético
dconstant = 2.64e-3;
LMNMDM = dconstant*BI_Tesla;

%%% TOTALES
XYZD = XYZSRP + XYZAERO;
LMND = LMNSRP + LMNAERO + LMNMDM;