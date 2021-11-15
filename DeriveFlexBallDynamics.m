syms x y phi theta psi a1 a2
syms xd yd phid thetad psid a1d a2d
syms xdd ydd phidd thetadd psidd a1dd a2dd

syms s_m b_m r d gz b_x b_y b_z

syms s_Jxx s_Jyy s_Jzz s_Jxy s_Jxz s_Jyz % Sphere: inertia matrix, center of the sphere
syms b_Jxx b_Jyy b_Jzz b_Jxy b_Jxz b_Jyz % Ballast: inertia matrix, center of the pendulum joint

syms s_Gx s_Gy s_Gz % Sphere: vector of first mass moments
syms b_Gx b_Gy b_Gz % Ballast: vector of first mass moments

% Joint variables, velocities and accelerations (8-var state vector)
gamma = [x; y; phi; theta; psi; a1; a2];
gammad = [xd; yd; phid; thetad; psid; a1d; a2d];
gammadd = [xdd; ydd; phidd; thetadd; psidd; a1dd; a2dd];

% Variables for chainDiff function
vars = [gamma; gammad];
dvars = [gammad; gammadd];

% Sphere frame
s.G = [s_Gx; s_Gy; s_Gz];
s.J = [s_Jxx s_Jxy s_Jxz; ...
       s_Jxy s_Jyy s_Jyz; ...
       s_Jxz s_Jyz s_Jzz];
   
% Ballast frame
b.G = [b_Gx; b_Gy; b_Gz];
b.J = [b_Jxx b_Jxy b_Jxz; ...
       b_Jxy b_Jyy b_Jyz; ...
       b_Jxz b_Jyz b_Jzz];

% FK: I --> s --> b
   
% .R: position of the the frame from the previous frame it's attached to
% .T: transformation from frame to previous frame 

% Sphere frame
s.R = [x; y; r];  % I -> s
s.T = rotx(phi)*roty(theta)*rotz(psi); % I -> s

% Ballast frame
b.R = [b_x; b_y; b_z]; % s -> b
b.T = rotx(a1)*roty(a2); % s -> b

% Define positions and velocities

IIRs = s.R;
% IIRsdot = jacobian(IIRs, gamma)*gammad;
IIRsdot = chainDiff(IIRs, vars, dvars);
ITs = s.T;
ITsdot = chainDiff(ITs, vars, dvars);

IIRb = IIRs + ITs*b.R;
IIRbdot = chainDiff(IIRb, vars, dvars);
ITb = ITs*b.T;
ITbdot = chainDiff(ITb, vars, dvars);

% Compute Kinetic and Potential Energies
s.KE = KineticEnergy(IIRsdot, s_m, ITsdot, s.G, s.J);
s.PE = PotentialEnergy(IIRs, s_m, ITs, s.G, gz);

b.KE = KineticEnergy(IIRbdot, s_m, ITbdot, b.G, b.J);
b.PE = PotentialEnergy(IIRb, s_m, ITb, b.G, gz);

KE = simplify(s.KE + b.KE);
PE = simplify(s.PE + b.PE);

Sk_w = simplify(ITsdot * ITs.');
IswI = unskew(Sk_w);
Iv = r*cross(IswI, [0;0;1]); % <-- Constraint equation

% Pfaffian matrix: multiples by gammad to get 0 for constraint
A = jacobian(gammad(1:2,1) - Iv(1:2,1), gammad)
Adot = chainDiff(A, vars, dvars)

% Compute Lagrangian of the system
L = KE - PE;

% Compute the system mass matrix
dL_dgammad = vectPartial(L,+ gammad);
dL_dgamma = vectPartial(L, gamma);

F = vectChainDiff(dL_dgammad, vars, dvars) - dL_dgamma;

H = simplify(jacobian(jacobian(KE, gammad).', gammad))
D = simplify(jacobian(jacobian(KE, gammad).', gamma)*gammad - jacobian(KE, gamma).')
G = simplify(jacobian(PE, gamma).')

