function [BfieldNav,pqrNav,ptpNav] = Navigation(BfieldMeasured,pqrMeasured,ptpMeasured);
global BfieldNavPrev pqrNavPrev ptpNavPrev Bdot
s = 0.3; %% s nos dice que tanto nos fiamos de las mediciones del sensor

if BfieldNavPrev(1,1) == -99
    BfieldNav = BfieldMeasured;
    pqrNav = pqrMeasured;
    ptpNav = ptpMeasured;
else 
    %%%Complimentary filter (fórmula teoría)
    BiasEstimate = [0;0;0];
    BfieldNav = BfieldNavPrev*(1-s) + s*(BfieldMeasured-BiasEstimate);
    pqrBiasEstimate = [0;0;0];
    pqrNav = pqrNavPrev*(1-s) +s*(pqrMeasured-pqrBiasEstimate);
    ptpBiasEstimate = [0;0;0];
    ptpNav = ptpNavPrev*(1-s) + s*(ptpMeasured-ptpBiasEstimate);
    sensor_params
    Bdot = (BfieldNav - BfieldNavPrev)/nextSensorUpdate;
end
BfieldNavPrev = BfieldNav; 
pqrNavPrev = pqrNav;
ptpNavPrev = ptpNav;