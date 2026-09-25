function dstatedt = Satellite(t,state)
%%%stateinitial = [x0;y0;z0;xdot0;ydot0;zdot0];
global BB B_ECI magneticUTC0 invI I Is m nextMagUpdate lastMagUpdate lastSensorUpdate nextSensorUpdate
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

%% Modelo del Campo Magnético: World Magnetic Model 2025
if t >= lastMagUpdate
    lastMagUpdate = lastMagUpdate + nextMagUpdate;
    %--------------------------------------------------------------
    % Tiempo absoluto de la simulación
    %--------------------------------------------------------------
    utcDateTime = magneticUTC0 + seconds(t);
    utc = datevec(utcDateTime);
    %--------------------------------------------------------------
    % ECI -> coordenadas geodésicas
    %
    % lla = [latitud, longitud, altura]
    % latitud y longitud en grados
    % altura en metros
    %--------------------------------------------------------------

    lla = eci2lla(r_pos',utc);

    latitude  = lla(1);
    longitude = lla(2);
    altitude  = lla(3);

    %--------------------------------------------------------------
    % WMM2025
    %
    % XYZ = [North; East; Down]
    % unidades: nT
    %--------------------------------------------------------------
    XYZ = wrldmagm( ...
        altitude, ...
        latitude, ...
        longitude, ...
        2025, ...
        '2025');
    B_NED = XYZ;
    C_ECEF_NED = dcmecef2ned(latitude,longitude);
    B_ECEF = C_ECEF_NED' * B_NED;
    C_ECI_ECEF = dcmeci2ecef('IAU-2000/2006',utc);
    B_ECI = C_ECI_ECEF' * B_ECEF;
end

%%% ECI -> Body
BB = Body2ECI_quat(q0123)' * B_ECI;

%%% Conversión de nT a T
BB = BB*1e-9;

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
Imax = maxCurrent/1000;
Ipeak = max(abs(current));

if Ipeak > Imax
    current = current * (Imax/Ipeak);
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