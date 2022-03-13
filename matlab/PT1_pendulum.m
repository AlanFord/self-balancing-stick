function [totalMass, length, rInertia, inertia] = PT1_pendulum(simpleRotor)
%% Inverted Pendulum Constants
%brief:  Calculate  the various physical constants of the inverted pendulum
%
%inputs:
% simpleRotor: logical variable indicating a simpleRotor (true) or
%              the standard rotor (false)
%
% outputs:
% totalMass: total mass of the pendulum, motors, and rotors in kg
% length:    Length of the pendulum in meters
% rInertia:  rotor moment of inertia = kg m^2
% inertia:   pendulum moment of inertia = kg m^2
% 
%% Section 1 Input Data

% Masses
mmountMass = 0.00615; %[kg] - mass of a single motor mount
motorMass = 0.0943; %[kg] - mass of a single brushed motor
hubMass = 0.00625; %[kg] - mass of the motor adapter hub
if ~simpleRotor   
    wheelMass = 0.0939136; %[kg] - mass of a single normal reaction wheel
else
    wheelMass = 0.175; %[kg] - mass of a single simple reaction wheel
end
rodMass = 0.0144; %[kg] - mass of the shaft
gmountMass = 0.0024; %[kg] - mass of the gyro mount
gblockMass = 0.0363; %[kg] - mass of the gripper block
% 22 gauge wire, 1.945lb/1000ft, or 0.88223716kg/304.8m, or 0.00289448kg/m
wireMass = 0.060; %[kg] - mass of the motor and gyro wiring
gyroMass = 0.003; %[kg] - mass of the gyro pcb, Adafruit estimate

% Inertial Data from Fusion360
if ~simpleRotor   
    wInertia = 1.595E-4; %[kg m^2] inertia of a single normal rotor wheel around the midline
else
    wInertia = 1.585E-4; %[kg m^2] inertia of a single simple rotor wheel around the midline
end

% Dimensions
motorLength = 0.05; %[m] length of motor and gearbox assembly
motorDiameter = 0.025; %[m] diameter of motor and gearbox assembly
shaftLength = 0.35; %[m] length of the entire pendulum shaft
motorHeight = 0.3; %[m] location of the motor assembly on the pendulum

%% Section 2 Mass Calculations
% Calculate the total mass of the reaction wheel pendulum

mAssemblyMass = mmountMass + motorMass + hubMass + wheelMass;

totalMass = rodMass+(2*mAssemblyMass)+gmountMass+gblockMass+wireMass ...
    +gyroMass;

%% Section 3 Rotor Assembly Moment of Inertia
% The objective here is to calculate the moment of inertia of the
% motor/rotor assembly around the motor shaft.

% calculate the moment of inertia of the entire motor/rotor assembly around
% the motor shaft
% assume half of the motor mass is spinning and is uniformly distributed in
% the motor volume
mInertia = 1/2*(motorMass+hubMass)*(motorDiameter/2)^2; %[kg m^2] motor moment of interia
rInertia = mInertia + wInertia; %[kg m^2] motor/gear/rotor inertia around motor shaft

%% Section 4 Pendulum Moment of Inertia
% The objective here is to calculate the moment of intertia of the
% pendulum assembly around the pendulum pivot point.

gyroHeight = shaftLength; %[m] location of the gyro on the pendulum
sInertia = 1/3*(rodMass+wireMass)*shaftLength^2; %[kg m^2] moment of the shaft

% assume smaller components are point masses
% use the parallel axis theorem to calculate the rotor assembly interia
% around the pivot point
inertia = sInertia + rInertia + ...
          2*(mAssemblyMass+gblockMass)*motorHeight^2 + ...
          (gmountMass+gyroMass)*gyroHeight^2;
      
%% Section 4 Pendulum Center of Mass

length = shaftLength/2 + ...
          2*(mAssemblyMass+gblockMass)*motorHeight + ...
          (gmountMass+gyroMass)*gyroHeight;

%% Section 5 Printing Results
fprintf('Part 1: Calculating Pendulum Data\n');
if (simpleRotor) phrase = ' simple rotor';
else phrase = 'standard rotor';
end
fprintf(strcat('The rotor is modeled as a', phrase,'\n'));
fprintf('total mass of the pendulum = %f kg \n',totalMass);
fprintf('distance from the pivot point to the pendulum center of mass = %f m \n', length)
fprintf('rotor moment of inertia = %f kg m^2 \n',rInertia);
fprintf('pendulum moment of inertia = %f kg m^2 \n',inertia);
fprintf('\n');







