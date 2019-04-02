%% Open Loop Transfer Function
% Calculate the various physical constants of the inverted pendulum
%% Section 1 Input Data
[mass, length, rotor_inertia, inertia] = pendulum();

[R, tConstant, Kt, frictionFactor] = motorMeasured();

g = 9.80665; % m/s/s

%% Section 2 Open Loop Transfer Coefficients

% assuming no friction

r = 1/4.5;   %PD zero location at 1/4, gain of r
%K=0.15;       % rotor feedback
K=0.075;       % rotor feedback

af = 0;
bf = 1;
cf = 0;
df = -(mass*g*length/inertia);
ef = 1/inertia;

ab = 0;
bb = 0;
cb = 1;
db = (frictionFactor*R-(K*Kt)+Kt^2)/rotor_inertia/R;
eb = Kt/R;

%Plot the root locus from the open-loop transfer function,
% including PD feeback and feedback from the rotor velocity

s=tf('s');
sysForward = ef/(af*s^3+bf*s^2+cf*s+df);
sysBackward = (s*eb)*(r*s+1)/(ab*s^3+bb*s^2+cb*s+db);
sys = sysForward*sysBackward
rlocus(sys)
axis('equal');
axis([-8 8 -8 8]);
grid on
%[k, poles] = rlocfind(sys)

% now let's look at the unit step response
