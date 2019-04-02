%% Open Loop Transfer Function
% Calculate the various physical constants of the inverted pendulum
%% Section 1 Input Data
[mass, length, rotor_inertia, inertia] = pendulum();

[R, tConstant, Kt, frictionFactor] = motorMeasured();

g = 9.80665; % m/s/s

%% Section 2 Open Loop Transfer Coefficients

% assuming no friction

%r = 1/4.5;   %PD zero location at 1/4, gain of r
%K=0.15;       % rotor feedback
%K=0.075;       % rotor feedback

r=0;
K=0;

%Plot the root locus from the open-loop transfer function,
% including PD feeback and feedback from the rotor velocity

s=tf('s');
Ptf = 1/(inertia*s^2-(mass*g*length));
Rtf = rotor_inertia*s;
Ctf = 1+r*s;
Mtf = (Kt/R)/(s*rotor_inertia + ((Kt^2)/R -(Kt*K)/R + frictionFactor));
sysForward = Ptf;
sysBackward = Rtf*Ctf*Mtf;
sys = sysForward*sysBackward
figure(1)
rlocus(sys)
axis('equal');
axis([-8 8 -8 8]);
grid on


%Re-plot the root locus using a PD feedback constant r = 1/4.5

%r = 1/4.5;   %PD zero location at 1/4, gain of r
r = 1/3.5;   %PD zero location at 1/4, gain of r
Ctf = 1+r*s;
sysBackward = Rtf*Ctf*Mtf;
sys = sysForward*sysBackward
figure(2)
rlocus(sys)
axis('equal');
axis([-8 8 -8 8]);
grid on


%Re-plot the root locus using a PD feedback constant r = 1/4.5
% and a rotor velocity coefficient of 0.075

% K=0.075;       % rotor feedback
K=0.1;       % rotor feedback
Mtf = (Kt/R)/(s*rotor_inertia + ((Kt^2)/R -(Kt*K)/R + frictionFactor));
sysBackward = Rtf*Ctf*Mtf;
sys = sysForward*sysBackward
figure(3)
rlocus(sys)
axis('equal');
axis([-8 8 -8 8]);
grid on
pause


close all
rr=rlocus(sys,266)
%[k, poles] = rlocfind(sys)
