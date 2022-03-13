function [R,tConstant, Kt, frictionFactor] = PT2_motorMeasured()
%% Motor Constants
%Brief: Provides constants for the electric motors.  The constants are
%       measured data specific to the Pololu 25D 4.4:1 metal gear motor.
%% Section 1 Input Data
L = 6.004E-3; % Henry
R = 7.9;     % Ohm
Kv = 0.0636;  % V sec
Kt = Kv;      % Nm/A
tConstant = L/R; % sec
frictionFactor = 1.20E-5; % Nm/(rad/sec)

%% Section 2 Printing Results
fprintf('Part 2: Specifying Electric Motor Parameters\n');
fprintf('inductance of the motor = %f H \n',L);
fprintf('resistance of the motor = %f ohm \n', R)
fprintf('motor voltage constant = %f V/s \n',Kv);
fprintf('motor torque constnt = %f N m/A \n',Kt);
fprintf('motor friction factor = %f N m sec \n',frictionFactor);
fprintf('\n');
