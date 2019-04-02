function [R,tConstant, Kt, frictionFactor] = motorMeasured()
%% Motor Constants
%  This is all from data published by the seller.  
%  See motor-measured.m for measured data.

%% Section 1 Input Data
L = 5.994E-3; % Henry
R = 5.82;     % Ohm
Kv = (11.95-(5.82*0.153))/1583.77 * 60 /(2*pi());  % V/sec^-1
Kt = Kv;      % Nm/A
tConstant = L/R; % sec
frictionFactor = 1.20E-5; % Nm/(rad/sec)