% estimate the inertia of a carbon steel reaction wheel

% density of carbon steel
density = 7.85 % g/cc
density = density /1000 * 1e6 % kg/m^3
diameter = 0.090  % m
thickness = 1/8 *25.4 /1000 % m
radius = diameter/2
mass = pi()*(radius^2)*thickness * density

% moment of inertial around centerline of a disk is
% 1/2 m r^2
csInertia = 1/2*mass*radius^2