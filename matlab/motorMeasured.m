function [R,tConstant, Kt, frictionFactor] = motorMeasured()
%% Motor Constants
%  This is all from data published by the seller.  
%  See motor-measured.m for measured data.

%% Section 1 Input Data
L = 6.004E-3; % Henry
R = 7.9;     % Ohm
Kv = 0.0636  % V sec
Kt = Kv;      % Nm/A
tConstant = L/R; % sec
frictionFactor = 1.20E-5; % Nm/(rad/sec)