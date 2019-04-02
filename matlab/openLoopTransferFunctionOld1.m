%% Open Loop Transfer Function
% Calculate the various physical constants of the inverted pendulum
%% Section 1 Input Data
[mass, length, rotor_inertia, inertia] = pendulum();

[R, tConstant, Kt, frictionFactor] = motorMeasured();

g = 9.80665; % m/s/s

%% Section 2 Open Loop Transfer Coefficients

% assuming no friction

r = 1/4.5;   %PD zero location at 1/4, gain of r
K=0.15;       % rotor feedback 
a = 1;
b = (frictionFactor*R-(K*Kt)+Kt^2)/(rotor_inertia*R);

c = -(mass*g*length/inertia);

d = -(mass*g*length/inertia)*(frictionFactor*R-(K*Kt)+Kt^2)/rotor_inertia/R;


e = Kt/(inertia*R);


s=tf('s');
sys = (s*e)*(r*s+1)/(s^3+b*s^2+c*s+d)
rlocus(sys)
axis('equal');
axis([-8 8 -8 8]);
grid on
%[k, poles] = rlocfind(sys)

open_loop_poles = pole(sys)

% now let's look at the unit step response
