%% Parámetros de los sensores
nextSensorUpdate = 0.1;
fsensor = 1; % Frecuencia de muestreo (Hz)
% Magnetómetro
MagBias = [1e-7; -2e-7; 1.5e-7]; % T
MagNoise = 1e-6; % T

% Giróscopo
AngBias = [1e-4; -2e-4; 1e-4]; % rad/s
AngNoise = 1e-4; %rad/s

% Sensor de actitud (Startracker)
EulerBias = [0.2; -0.1; 0.15]*pi/180;
EulerNoise = 0.05*pi/180;