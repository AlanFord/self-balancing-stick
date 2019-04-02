% start by computing the open loop transfer functions with feedback
openLoopTransferFunction;

% Let's calculate the response of the pendulum angle to a step change
%  in the applied force.
opt = stepDataOptions('StepAmplitude',0.05);
%gain=266;
gain=350;
T = feedback(sysForward,sysBackward*gain,-1)

t = 0:0.01:6;
figure(1)
step(T,t,opt)

% Now, the response to a shove
figure(2)
impulse(T)

% Let's reconfigure and look at the step response of the rotor velocity
sysForward = Ptf*Ctf*Mtf;
sysBackward = Rtf;
T = feedback(sysForward,sysBackward*gain,-1)
figure(3)
step(T,t,opt)
figure(4)
impulse(T)
