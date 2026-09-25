%%% Inercia y masa del satélite
ms = 2.6; %% masa del satélite en kg
lx = 10/100; %%meters
ly = 10/100; %%meters
lz = 20/100; %%meters
l = [lx;ly;lz];
lsort = sort(l);
Amax = lsort(2)*lsort(3);
lmax = l(3);
CD = 1.0;

Is = (ms/12)*[(ly^2+lz^2) 0 0 ;0 (lx^2+lz^2) 0;0 0 (lx^2+ly^2)]; %% Inercia en kg*m^2

%%% Parámetros de las ruedas de reacción
reaction_wheel_params

%%% Suma reaction wheels + resto del satélite
m = ms + 3*m_rws;
I = Is + Ir1Bcg + Ir2Bcg + Ir3Bcg;
invI = inv(I);