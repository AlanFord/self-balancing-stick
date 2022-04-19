%% Open Loop Transfer Function
%Brief: Calculate the various physical constants of the inverted pendulum

%% Section 1 Input Data
[mass, length, rotor_inertia, inertia] = PT1_pendulum(false);

[R, tConstant, Kt, frictionFactor] = PT2_motorMeasured();

g = 9.80665; % m/s/s

%% Section 2 Open Loop Transfer Coefficients

% Option 1: P feedback but no rotor velocity feedback
if exist('p','var') == 0
    p=0;
end
if exist('q','var') == 0
    q=0;
end
if exist('Kw','var') == 0
    Kw=0;
end
%p=0;
%q=0;
%Kw=0;

% Option 2: PD but no rotor velocity feedback
%p = 1/4.5;   % PD zero location at 1/4, gain of r
%q=0;
%Kw=0;

% Option 3: PD but no rotor velocity feedback
%p = 1/4.5;   % PD zero location at 1/4, gain of r
%q=0;
%Kw=0.075;   % rotor feedback

s=tf('s');
Ptf = 1/(inertia*s^2-(mass*g*length));
Rtf = rotor_inertia*s; % the negative sign is implemented using a negative feedback model
%Mtf = (Kt/R/rotor_inertia)/(s + ((1/rotor_inertia)*((Kt^2)/R -(Kt*Kw)/R + frictionFactor)));
Mtf = (Kt/R)/(rotor_inertia*s + ((Kt^2)/R -(Kt*Kw)/R + frictionFactor));
Ctf = 1+(q/s)+(p*s);
sysForward = Ptf;
sysBackward = Rtf*Ctf*Mtf;
sys = sysForward*sysBackward
[z,gain] = zero(sys)
P = pole(sys)
H = zpk(sys)
memo = tf(H)


%% Section 3 root locus
figure(1)
rlocus(sys)
axis('equal');
%axis([-8 8 -8 8]);
%grid on
myTitle = title(['Parameters: p=' num2str(p) ', q=' num2str(q) ', Kw=' num2str(Kw)]); 
myTitle.FontSize = 12;

fprintf('Press any key to continue \n');
pause

%close all
rr=rlocus(sys,400)
