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
Ptf = 1/(inertia*s^2-(mass*g*length)); % pendulum tf
Rtf = rotor_inertia*s; % rotor tf % the negative sign is implemented using a negative feedback model
%Mtf = (Kt/R/rotor_inertia)/(s + ((1/rotor_inertia)*((Kt^2)/R -(Kt*Kw)/R + frictionFactor)));
Mtf = (Kt/R)/(rotor_inertia*s + ((Kt^2)/R -(Kt*Kw)/R + frictionFactor)); % motor tf
Ctf = 1+(q/s)+(p*s); % controller tf
sysForward = Ptf;  % forward tf
sysBackward = Rtf*Ctf*Mtf; % backward tf
sys = sysForward*sysBackward  % full 'open loop' tf
[z,gain] = zero(sys) % determine zeros and gain of open loop
P = pole(sys)  % poles of open loop
H = zpk(sys) % zeros, poles, and gain of open loop
memo = tf(H) % full poly form of open loop



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
clear gain
gain = 347;
%gain = 370;
fprintf(['Parameters: p=' num2str(p) ', q=' num2str(q) ', Kw=' num2str(Kw) ', gain=' num2str(gain) '\n'])
fprintf('Kp= %f Ki= %f Kd= %f Kw= %f \n',gain, gain*q, gain*p, gain*Kw)
rr=rlocus(sys,gain)
fprintf('--> End of open loop calculations <-- \n')

%% Section 4 state space conversion
%myCl = feedback(sys,gain)
%mySS = ss(myCl)