%% Inicialización
clear
clc
close all
tic

global BB m I Is invI mu nextMagUpdate lastMagUpdate lastSensorUpdate nextSensorUpdate
global BfieldMeasured pqrMeasured ptpMeasured BfieldNav pqrNav ptpNav 
global BfieldNavPrev pqrNavPrev ptpNavPrev current Ir1Bcg Ir2Bcg Ir3Bcg n1 n2 n3
global maxSpeed maxAlpha Ir1B Ir2B Ir3B rwalphas Bdot DETUMBLE
global fsensor MagBias AngBias EulerBias R Amax lmax CD
global MagNoise AngNoise EulerNoise IrR Jinv rwSATURATED


disp('Simulation started')
nextMagUpdate = 1;
lastMagUpdate = 0; 

%%% Modelo IGRF (campo magnético) (FUTURO CAMBIO)
addpath 'igrf/'

%%% Parámetros del planeta
planet_params

%%% Masas e Inercias
inertia

%% Condiciones Iniciales
%%% Condiciones Iniciales de Posición y Velocidad (Dinámica Traslacional)
altitude = 600*1000;
x0 = R + altitude;
y0 = 0;
z0 = 0;
inclination = 56*pi/180;
semi_major = norm([x0;y0;z0]);
norm_radio = semi_major;
vcircular = sqrt(mu/norm_radio);
xdot0 = 0;
ydot0 = vcircular*cos(inclination);
zdot0 = vcircular*sin(inclination);

position0 = [x0; y0; z0];
velocity0 = [xdot0; ydot0; zdot0];

%%% Condiciones iniciales para Actitud y Velocidad Angular (Dinámica Rotacional)
phi0 = 0;
theta0 = 0;
psi0 = 0;
ptp0 = [phi0; theta0; psi0];
q0123_0 = EulerAngles2Quaternions(ptp0);
p0 = 0.08;
q0 = -0.02;
r0 = 0.02;
pqr0 = [p0; q0; r0];

%%% Condiciones iniciales de las Ruedas de Reacción (rws)
w1230 = zeros(3,1);

%%% State final con todas las condiciones iniciales
state = [position0; velocity0; q0123_0; pqr0; w1230];

%% Párametros de tiempo
period = 2*pi/sqrt(mu)*semi_major^(3/2);
number_of_orbits = 1;
tfinal = period*number_of_orbits;  
timestep = 0.1; 
next = 10;
tout = 0:timestep:tfinal;
stateout = zeros(length(tout),length(state));

%% Integrador

%%% Aclaración: Se podría usar ode45 pero tiene un paso variable. Para la
%%% información de los sensores merece la pena un paso fijo

%%% Campo magnético
BfieldOut = zeros(length(tout),3); % real
BfieldMeasuredOut = zeros(length(tout),3); % medido por los sensores
BfieldNavOut = zeros(length(tout),3); % filtrado

% Velocidad angular
pqrMeasuredOut = zeros(length(tout),3); 
pqrNavOut = zeros(length(tout),3);

% Ángulos de Euler
ptpMeasuredOut = zeros(length(tout),3); 
ptpNavOut = zeros(length(tout),3);

% Magnetorquer currents 
magCurrentOut = zeros(length(tout),3);

rwAlphaOut = zeros(length(tout),3);
BfieldNavPrev = [-99;0;0];
pqrNavPrev = [0;0;0];
ptpNavPrev = [0;0;0];
Bdot = [0;0;0];
current = [0;0;0];
rwalphas = [0;0;0];
rwSATURATED = 0;
DETUMBLE = 1;
omega_threshold = 0.01; % rad/s
detumble_time = 10; % segundos
detumble_start_time = NaN;


%%% Sensor 
lastSensorUpdate = 0;
sensor_params

%%%%Control Parameters
lastControl = -timestep;
nextControl = 0.1;

%%%Print Next
lastPrint = 0;

k1 = Satellite(tout(1),state); %Inicializar para poder almacenar soluciones
for idx = 1:length(tout)
    %%% Save the current state
    stateout(idx,:) = state.';  %% Hay que transponer la matriz porque state es un vector volumna y queremos stateout en vector fila
   
    %%%Save the Current
    magCurrentOut(idx,:) = current.';

    %%%%Save reaction wheel acceleration
    rwAlphaOut(idx,:) = rwalphas.';

    %%%Save magnetic field
    BfieldOut(idx,:) = BB.';
    BfieldMeasuredOut(idx,:) = BfieldMeasured.';
    BfieldNavOut(idx,:) = BfieldNav.';

    %%%Save angular velocity
    pqrMeasuredOut(idx,:) = pqrMeasured.';
    pqrNavOut(idx,:) = pqrNav.';

    %%%Save Euler Angles
    ptpMeasuredOut(idx,:) = ptpMeasured.';
    ptpNavOut(idx,:) = ptpNav.';

    %%% Print progress
    if tout(idx) > lastPrint
        disp(['Time = ', num2str(tout(idx))])
        lastPrint = lastPrint + next;
    end
    
    %%% DETUMBLE LOGIC
    if DETUMBLE
        omega = norm(pqrNav);
        if omega < omega_threshold
            if isnan(detumble_start_time)
                detumble_start_time = tout(idx);

            elseif (tout(idx) - detumble_start_time) >= detumble_time
                DETUMBLE = 0;
                disp('Detumble completed - switching to attitude control')
            end
        else
            detumble_start_time = NaN;
        end
    end

    if tout(idx) > lastControl
        %%% Asumo que tengo un feedback perfecto de las RWs
        w123 = state(14:16);
        [current,rwalphas] = Control(BfieldNav,pqrNav,ptpNav,w123);
        lastControl = lastControl + nextControl;
    end

    %%% Las siguientes 4 funciones son las que determinan el integrador RK4
    k1 = Satellite(tout(idx),state);
    k2 = Satellite(tout(idx)+timestep/2, state+k1*timestep/2);
    k3 = Satellite(tout(idx)+timestep/2, state+k2*timestep/2);
    k4 = Satellite(tout(idx)+timestep, state+k3*timestep);
    k = (1/6)*(k1+2*k2+2*k3+k4);
    state = state + k*timestep;
end

%%%Save original State
stateout_original = stateout;

disp('Simulation Completed')

%% Resultados
%%% Paso el estado de metros a kilometros para facilitar la representación
stateout(:, 1:6) = stateout(:, 1:6)/1000;

%%% Extraer el vector posición, cuaterniones y velocidad angular
positionOut = stateout(:,1:3);
xout = positionOut(:,1); 
yout = positionOut(:,2); 
zout = positionOut(:,3);
q0123out = stateout (:, 7:10);
ptpout = Quaternions2EulerAngles(q0123out);
pqrout = stateout (:, 11:13);
w123out = stateout(:,14:16);

%% Current
%%% Corriente de las ruedas de reacción
mag_current_total = sum(abs(magCurrentOut),2); 

%%% Corriente de las ruedas de reacción
current_rwa = Amps2Alpha * rwAlphaOut;
rw_current_total = sum(abs(current_rwa),2);

%%% Corriente Total 
total_current = mag_current_total + rw_current_total;

%% PLOTS
%%% "Modelo" Tierra para representación 3D
[X,Y,Z] = sphere;
X = X*R/1000;
Y = Y*R/1000;
Z = Z*R/1000; 

%%% Plot Posición del Satélite por coordenadas
fig1 = figure();
set(fig1,'color','white')
plot(tout,xout,'b-','LineWidth',2)
hold on
grid on
plot(tout,yout,'r-','LineWidth',2)
plot(tout,zout,'g-','LineWidth',2)
xlabel('Tiempo (s)')
ylabel('Posición (km)')
legend('X','Y','Z')

%%% Plot órbita 3D
fig2 = figure();
set(fig2,'color','white')
plot3(xout,yout,zout,'b-','LineWidth',4)
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
hold on
surf(X,Y,Z,'EdgeColor','none')
axis equal

%%% Plot Campo magnético
fig3 = figure();
set(fig3,'color','white')
pB = plot(tout,BfieldOut,'LineWidth',2);
hold on
grid on
pBm = plot(tout,BfieldMeasuredOut,'-s','LineWidth',1.5);
pBN = plot(tout,BfieldNavOut,'--','LineWidth',1.5);
xlabel('Tiempo (s)')
ylabel('Campo Magnético (T)')
legend([pB(1),pBm(1),pBN(1)],'Actual','Measured','Nav')

%%% Plot norma del campo magnético
Bnorm = vecnorm(BfieldOut,2,2);
fig4 = figure();
set(fig4,'color','white')
plot(tout,Bnorm,'LineWidth',2)
xlabel('Tiempo (s)')
ylabel('Norm del campo magnético (T)')
grid on

%%% Plot ángulos de Euler
fig5 = figure();
set(fig5,'color','white')
p1 = plot(tout,ptpout*180/pi,'-','LineWidth',2);
hold on
p2 = plot(tout,ptpMeasuredOut*180/pi,'-s','LineWidth',1.5);
p3 = plot(tout,ptpNavOut*180/pi,'--','LineWidth',1.5);
grid on
xlabel('Time (sec)')
ylabel('Ángulos de Euler (deg)')
legend('Phi','Theta','Psi')
legend([p1(1),p2(1),p3(1)],'Actual','Measured','Nav')

%%% Plot velocidad angular del satélite
fig6 = figure();
set(fig6,'color','white')
p1=plot(tout,pqrout,'-','LineWidth',2);
hold on
p2=plot(tout,pqrMeasuredOut,'-s','LineWidth',1.5);
p3=plot(tout,pqrNavOut,'--','LineWidth',1.5);
grid on
xlabel('Tiempo (s)')
ylabel('Velocidad Angular (rad/s)')
legend([p1(1),p2(1),p3(1)],'Actual','Measured','Nav')

%%% Plot de la corriente de los magnetopares por separado
fig7 = figure();
set(fig7,'color','white')
plot(tout,magCurrentOut*1000,'LineWidth',2)
grid on
xlabel('Tiempo (s)')
ylabel('Corriente Magnetopares (mA)')
legend('X','Y','Z')

%%% Plot corriente total de los tres magnetopares
fig8 = figure();
set(fig8,'color','white')
plot(tout,mag_current_total*1000,'LineWidth',2)
grid on
xlabel('Tiempo (s)')
ylabel('Corriente TOTAL Magnetopares (mA)')

%%% Plot de las aceleraciones de las RWs
fig9 = figure();
set(fig9,'color','white')
plot(tout,rwAlphaOut,'LineWidth',2)
grid on
xlabel('Tiempo (s)')
ylabel('Aceleración Angular RWs (rad/s^2)')
legend('X','Y','Z')

%%% Plot velocidades angulares de las RWs
fig10 = figure();
set(fig10,'color','white')
plot(tout,w123out,'LineWidth',2)
grid on
xlabel('Tiempo (s)')
ylabel('Velocidad Angular RWs (rad/s)')
legend('X','Y','Z')

%%% Plot de la corriente de las RWs por separado
fig11 = figure();
set(fig11,'color','white')
plot(tout,current_rwa*1000,'y','LineWidth',2)
grid on
xlabel('Tiempo (s)')
ylabel('Corriente RWs (mA)')
legend('X','Y','Z')

%%% Plot de la corriente de todas las RWs juntas
fig12 = figure();
set(fig12,'color','white')
plot(tout,rw_current_total*1000,'y','LineWidth',2)
grid on
xlabel('Tiempo (s)')
ylabel('Corriente TOTAL RWs (mA)')

%%% Plot corriente magnetopares + RWs
fig13 = figure();
set(fig13,'color','white')
plot(tout,total_current*1000,'y','LineWidth',2)
grid on
xlabel('Tiempo (s)')
ylabel('Corriente TOTAL (mA)')

toc