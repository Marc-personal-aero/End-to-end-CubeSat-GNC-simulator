function dstatedt = Satellite(t,state)
%%%stateinitial = [x0;y0;z0;xdot0;ydot0;zdot0];
global BB invI I Is m nextMagUpdate lastMagUpdate lastSensorUpdate nextSensorUpdate
global BfieldMeasured pqrMeasured BfieldNav pqrNav BfieldNavPrev pqrNavPrev current
global Ir1Bcg Ir2Bcg Ir3Bcg n1 n2 n3 rwalphas maxSpeed maxAlpha ptpMeasured
global ptpNavPrev ptpNav  fsensor rwSATURATED R Amax lmax CD 
global Ir1B Ir2B Ir3B MagBias AngBias EulerBias
global MagNoise AngNoise EulerNoise

%% Extracción del Estado
r_pos = state(1:3);
x = r_pos(1); 
y = r_pos(2); 
z = r_pos(3);
vel = state(4:6);
q0123 = state(7:10);
ptp = Quaternions2EulerAngles(q0123')';
pqr = state(11:13);
p = state(11); 
q = state(12); 
r = state(13);
w123 = state(14:16);

%% Cinemática Rotacional
PQRMAT = [0 -p  -q -r;
          p  0   r -q;
          q  -r  0  p;
          r  q  -p  0];
q0123dot = (1/2)*PQRMAT*q0123;

%% Modelo Gravitacional
planet_params
rho = norm(r_pos);
r_unit = r_pos/rho; 
Fgrav = -(G*M*m/rho^2)*r_unit; 

%% Modelo del Campo Magnético
if t>=lastMagUpdate
    lastMagUpdate = lastMagUpdate + nextMagUpdate;
    %%% Conversión de ECI cartesiano a latitud, longitud y altitud
    phiE = 0;
    thetaE = acos(z/rho);
    psiE = atan2(y,x);
    latitude = 90 - thetaE*180/pi;
    longitude = psiE*180/pi;
    altitude = (rho)/1000;    % km 
    [BN,BE,BD] =  igrf ('01-Jan-2020', latitude, longitude, altitude, 'geocentric');
    %%% Primero tenemos que convertir el sistema NED (North-East-Down) en ECI
    %%% (Earth-Centered-Inertial) mediante la matriz TIB programada en otra
    %%% subrutina. ¿FUTURO CAMBIO A OTRO IGRF?
    BNED = [BN;BE;-BD];  %%PCI tiene down como up
    %%% Para cambiar de NED a ECI se multiplica TIB*BNED
    B_ECI = Euler2DCM(phiE,thetaE+pi,psiE)*BNED;
    %%% Cambio de ECI a Body (la matriz de cambio está traspuesta para hacer el cambio correctamente)
    BB = Body2ECI_quat(q0123)'*B_ECI;
    %%Convertir de nanoteslas a teslas
    BB = BB*1e-9;
end

%% Sensor y Navegación
if t>=lastSensorUpdate
    %%% Bloque del Sensor
    lastSensorUpdate = lastSensorUpdate + nextSensorUpdate;
    [BfieldMeasured,pqrMeasured,ptpMeasured] = Sensor(BB,pqr,ptp);

    %%% Bloque de Navegación (FILTRADO)
    [BfieldNav,pqrNav,ptpNav] = Navigation(BfieldMeasured,pqrMeasured,ptpMeasured);
end

%% Magnetorquers
magnetorquer_params

%%% Añadir la Saturación
if sum(abs(current)) > maxCurrent/1000
    current = max(min(current, maxCurrent/1000), -maxCurrent/1000); %% min hace que un valor de current superior a 0.12A sea igual a 0.12 y max lo mismo pero al reves. 
end

%%% Momento magnético
muB = current*n*A;
%%% Momento magnético genreado por los magnetopares
LMN_magnetorquers = cross(muB,BB);

%% Ruedas de Reacción (RWs)
w123dot = [0;0;0];
for idx = 1:3
    %%% Deteccion de saturacion
    if abs(w123(idx)) > 0.8*maxSpeed && rwSATURATED == 0
        disp('Reaction Wheels have Saturated. Moving to desaturization scheme')
        rwSATURATED = 1;
    end
    %%% Limite de aceleracion de las ruedas
    if abs(rwalphas(idx)) > maxAlpha
        rwalphas(idx) = sign(rwalphas(idx))*maxAlpha;
    end
    %%% La aceleracion de la rueda viene siempre del controlador
    w123dot(idx) = rwalphas(idx);
end

LMN_RWs = Ir1B*w123dot(1)*n1 + Ir2B*w123dot(2)*n2 + Ir3B*w123dot(3)*n3;

%% Perturbaciones (Disturbance)
[XYZD,LMND] = Disturbance(rho-R,Amax,lmax,vel,CD,BB);

%% Momentos totales (magnetopares, ruedas de reacción y factores externos)
LMN = LMN_magnetorquers - LMN_RWs + LMND;

%% Dinámica traslacional
F = Fgrav;
accel = F/m;

%% Dinámica rotacional
w1 = w123(1);
w2 = w123(2);
w3 = w123(3);
H = Is*pqr + Ir1Bcg*w1*n1 + Ir2Bcg*w2*n2 + Ir3Bcg*w3*n3;
pqrdot = invI*(LMN - cross(pqr, H));

%% Devuelve el Derivative Vector (vector con velocidad y aceleración)
dstatedt = [vel; accel; q0123dot; pqrdot; w123dot];