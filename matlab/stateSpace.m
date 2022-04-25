%% State Space Model
%Brief: Create and test a state space model of the pendulum

%% Section 1 Input Data
[mass, length, rotor_inertia, inertia] = PT1_pendulum(false);

[R, tConstant, Kt, frictionFactor] = PT2_motorMeasured();

g = 9.80665; % m/s/s

% Option 1: P feedback but no rotor velocity feedback
if exist('p','var') == 0
    p=0;   % derivative coefficient
end
if exist('q','var') == 0
    q=0;   % integral coefficient
end
if exist('Kw','var') == 0
    Kw=0;  % rotor velocity coefficient
end

%% Section 2 Set up the matrixes
part1 = frictionFactor/rotor_inertia +Kt*Kt/rotor_inertia/R;
part2 = frictionFactor/inertia       +Kt*Kt/inertia/R;
part3 = mass*g*length;
Am = [-part1 0 part1;0 0 1; part2 part3 -part2]
clear part1 part2 part3;

part1 = Kt/rotor_inertia/R;
part2 = -Kt/inertia/R;
Bm = [part1; 0; part2]
clear part1 part2;

P = [1 0 -1; 0 1 0; 0 0 1];
A = P*Am*inv(P)
B=P*Bm

C = [1 0 0 ; 0 1 0]

%{
%% Section X Calculate the Coefficients
K2 = 700; % the proportional gain

K1 = K2*q;   % the integral gain
K3 = K2*p; % the differential gain
K4 = K2*Kw; % the rotor velocity gain
Kv = Kt;

B1 = Kt*K1/rotor_inertia/R;
B2 = Kt*K2/rotor_inertia/R;
B3 = Kt*K3/rotor_inertia/R;
B4 = Kt*(K4-Kv)/rotor_inertia/R - frictionFactor/rotor_inertia;

A1 = -Kt*K1/inertia/R;
A2 = mass*g*length/inertia - Kt*K2/inertia/R;
A3 = -Kt*K3/inertia/R;
A4 = frictionFactor/rotor_inertia - Kt*(K4-Kv)/rotor_inertia/R;

% in the form xDot = a*x + b*u
%             y = c*x
a = [0 1 0 0; 0 0 1 0; A1 A2 A3 A4; B1 B2 B3 B4];
b = transpose([0 0 1/inertia 0]);
%c = [K1 K2 K3 K4; K1/R K2/R K3/R (K4-Kv)/R; 0 1 0 0; 0 0 0 1];
c = [0 1 0 0];
sys = ss(a,b,c,0);
time = 0:0.01:6;
% figure(1)
%step(sys,time)
% step(sys)
% myTitle = title(['Step, Parameters: K1=' num2str(K1) ', K2=' num2str(K2) ', K3=' num2str(K3)]); 
% myTitle.FontSize = 12;
% ylabel('Pendulum angle (radians)');
%}
