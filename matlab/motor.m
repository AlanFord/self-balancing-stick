function [R,tConstant, Kt] = motor(L, V, stallTorque, stallCurrent, noloadSpeed, noloadCurrent)
%% Motor Constants
%  This is all from data published by the seller.  
%  See motor-measured.m for measured data.

%% Section 1 Input Data

% Electrical Data
% gear_ratio =4.4;
% R=27.07; %[ohm] - >>>>Inferred<<<<<
stallTorque = stallTorque * 0.0070615518333333; % convert to Nm
noloadSpeed = noloadSpeed/60*2*pi; % convert to radians/sec

%% Section 5 Motor Constants
% The objective here is to 
R = V / stallCurrent;   % doesn't match the measured value
Kt = stallTorque / stallCurrent;   % motor torque constant Nm/A

% Let's estimate Kv
Kv = (V - R*noloadCurrent) / noloadSpeed;  % motor voltage constant (V sec)/rad

%% Section 3 Motor Time Constant
% Calculate the motor time constant

tConstant = L/R; % time constant of the motor 

%% Section 6 Printing Results
fprintf('motor resistance = %f ohms \n',R)
fprintf('motor time constant = %f 1/sec \n',tConstant)
fprintf('motor stall torque = %f Nm \n',stallTorque)
fprintf('motor no-load speed = %f rad/sec \n', noloadSpeed)
fprintf('Kt = %f Nm/A \n',Kt)
fprintf('Kv = %f V*sec/rad \n',Kv)
end