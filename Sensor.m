function [BB_measured,pqr_measured,ptp_measured] = Sensor(BB,pqr,ptp)

global MagBias AngBias EulerBias
global MagNoise AngNoise EulerNoise

BB_measured = zeros(3,1);
pqr_measured = zeros(3,1);
ptp_measured = zeros(3,1);

for idx = 1:3
    BB_measured(idx) = BB(idx) + MagBias(idx) + MagNoise*randn;
    pqr_measured(idx) = pqr(idx) + AngBias(idx) + AngNoise*randn;
    ptp_measured(idx) = ptp(idx) + EulerBias(idx) + EulerNoise*randn;
end
