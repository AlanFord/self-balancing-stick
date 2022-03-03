%% Open Loop Transfer Function
% Calculate the various physical constants of the inverted pendulum
%% Section 1 Input Data
[mass, length, rotor_inertia, inertia] = pendulum();

[R, tConstant, Kt, frictionFactor] = motorMeasured();

g = 9.80665; % m/s/s

%% Section 2 Open Loop Transfer Coefficients

% assuming no friction

%p = 1/4.5;     %PD zero location at 1/4, gain of r
%Kw=0.15;       % rotor feedback
%Kw=0.075;      % rotor feedback

p=0;
q=0;
Kw=0;

%Plot the root locus from the open-loop transfer function,
% including PD feeback and feedback from the rotor velocity

s=tf('s');
Ptf = 1/(inertia*s^2-(mass*g*length));
Rtf = rotor_inertia*s; % the negative sign is implemented using a negative feedback model
Mtf = (Kt/R/rotor_inertia)/(s + ((1/rotor_inertia)*((Kt^2)/R -(Kt*Kw)/R + frictionFactor)));
Ctf = 1+(q/s)+(p*s);
sysForward = Ptf;
sysBackward = Rtf*Ctf*Mtf;
sys = sysForward*sysBackward
figure(1)
rlocus(sys)
axis('equal');
axis([-8 8 -8 8]);
grid on
title(['p=' num2str(p) ', q=' num2str(q) ', Kw=' num2str(Kw)]); 


%Re-plot the root locus using a PD feedback constant r = 1/4.5

p = 1/4.5;   % PD zero location at 1/4, gain of r
%p = 1/3.5;    % PD zero location at 1/4, gain of r
Ctf = 1+(q/s)+(p*s);
sysBackward = Rtf*Ctf*Mtf;
sys = sysForward*sysBackward
figure(2)
rlocus(sys)
axis('equal');
axis([-8 8 -8 8]);
grid on
title(['p=' num2str(p) ', q=' num2str(q) ', Kw=' num2str(Kw)]); 


%Re-plot the root locus using a PD feedback constant r = 1/4.5
% and a rotor velocity coefficient of 0.075

Kw=0.075;   % rotor feedback
% Kw=-0.1;       % rotor feedback
%Mtf = (Kt/R)/(s*rotor_inertia + ((Kt^2)/R -(Kt*Kw)/R + frictionFactor));
Mtf = (Kt/R/rotor_inertia)/(s + ((1/rotor_inertia)*((Kt^2)/R -(Kt*Kw)/R + frictionFactor)));
sysBackward = Rtf*Ctf*Mtf;
sys = sysForward*sysBackward
figure(3)
rlocus(sys)
axis('equal');
axis([-8 8 -8 8]);
grid on
title(['p=' num2str(p) ', q=' num2str(q) ', Kw=' num2str(Kw)]); 
fprintf('Press any key to continue \n');
pause


close all
rr=rlocus(sys,370)
%[k, poles] = rlocfind(sys)
