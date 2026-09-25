%%% Este código sirve como apoyo a la parte de Simulink. Tiene la función 
%%% todas las variables inciales y constantes del problema. 

clear; clc;

% Cargar constantes y parámetros
planet_params;          
inertia; 
reaction_wheel_params;
magnetorquer_params;
sensor_params;   
s = 0.3;  % Factor de confinanza de los sensores para el Complimentary Filter
k = 67200;

% Condiciones Iniciales Orbitales
altitude = 600*1000;
x0 = R + altitude; 
y0 = 0; 
z0 = 0;
inclination = 56*pi/180;
semi_major = norm([x0;y0;z0]);
vcircular = sqrt(mu/semi_major);

r0_eci = [x0; y0; z0];
v0_eci = [0; vcircular*cos(inclination); vcircular*sin(inclination)];

% Condiciones Iniciales de Actitud (Rotación)
ptp0 = [0; 0; 0]; 
q0 = EulerAngles2Quaternions(ptp0); % [q0; q1; q2; q3][cite: 1, 10]
pqr0 = [0.08; -0.02; 0.03];         % rad/s

% Tiempo de Simulación
period = 2*pi/sqrt(mu)*semi_major^(3/2);
num_orbitas = 5;
tfinal = period * num_orbitas; 