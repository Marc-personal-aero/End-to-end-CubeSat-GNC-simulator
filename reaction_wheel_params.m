%% Parámetros de las ruedas de reacción
m_rws = 0.15;  % kg
r_rws = 20/1000;  % m
h_rws = 30/1000;  % m

%%% Velocidad máxima
max_rpm = 7000; % rpm
maxSpeed = max_rpm*2*pi/60; % rad/s
rpm_desat = 1000;
desatSpeed = rpm_desat*2*pi/60;

%%% Torque Máximo
maxTorque = 0.007; % N*m

%%% Parámetros de consumo de Potencia
dc_voltage = 12.0; % Voltios (V)
peak_power = 10; % Vatios (W)

%%% Ratio de consumición de la potencia
Power2Alpha = 1; %%% Watts / rad/s^2
Amps2Alpha = 1; %%% Amps / rad/s^2

%%% Posición de las ruedas de reacción respecto cg
r1 = [4;0;0]/1000;
r2 = [0;4;0]/1000;
r3 = [0;0;4]/1000;

%%% Orientación de las ruedas de reacción expresado en Body Frame
n1 = [1;0;0];
n2 = [0;1;0];
n3 = [0;0;1];

%% Inercias

%%% Tensor de Inercia de las ruedas de reacción con sist de ref en las rws
%%% respecto el centro de las rws
% Ixx: momento de inercia respecto el eje de rotación
% Iyy = Izz: momentos de inercia perpendiculares al eje de rotación
Ixx_R = 0.5 * m_rws * r_rws^2;
Iyy_R = (m_rws/12) * (3*r_rws^2 + h_rws^2);

IrR = [Ixx_R   0        0;
        0    Iyy_R      0;
        0      0       Iyy_R];

%%% Máxima aceleración angular
maxAlpha = maxTorque/IrR(1,1);

%%% Tranformación de sistema de ref de las RWs a Body Frame del satélite (Axis RWs a Direction Cosine Matrix)
T1 = Axis2DCM(n1);
T2 = Axis2DCM(n2);
T3 = Axis2DCM(n3);

%%% Inercia de cada rueda en sistema de referencia del Satélite respecto
%%% al centro de cada rueda
Ir1B = T1*IrR*T1';
Ir2B = T2*IrR*T2';
Ir3B = T3*IrR*T3';

%%% J para Control
J = [Ir1B*n1,Ir2B*n2,Ir3B*n3];
Jinv = J'/(J*J');

%%% Inercia de cada rueda en sistema de ref del satelite respecto al CG del
%%% satélite aplicando Steiner
sr1 = skew(r1);
Ir1Bcg = Ir1B + m_rws*(sr1')*sr1;
sr2 = skew(r2);
Ir2Bcg = Ir2B + m_rws*(sr2')*sr2;
sr3 = skew(r3);
Ir3Bcg = Ir3B + m_rws*(sr3')*sr3;