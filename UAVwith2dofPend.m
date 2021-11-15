syms x y z phi theta psi a1 a2
syms xd yd zd phid thetad psid a1d a2d
syms xdd ydd zdd phidd thetadd psidd a1dd a2dd

syms uav_m b_m p1_m p2_m r d gz b_x b_y b_z

syms uav_Jxx uav_Jyy uav_Jzz uav_Jxy uav_Jxz uav_Jyz % UAV: inertia matrix, center of the sphere
syms p1_Jxx p1_Jyy p1_Jzz p1_Jxy p1_Jxz p1_Jyz % Pend1: inertia matrix, center of the pendulum joint
syms p2_Jxx p2_Jyy p2_Jzz p2_Jxy p2_Jxz p2_Jyz % Pend2: inertia matrix, center of the pendulum joint


syms uav_Gx uav_Gy uav_Gz % UAV: vector of first mass moments
syms p1_Gx p1_Gy p1_Gz % Pend1: vector of first mass moments
syms p2_Gx p2_Gy p2_Gz % Pend1: vector of first mass moments

% Joint variables, velocities and accelerations (8-var state vector)
gamma = [x; z; theta; a1; a2];
gammad = [xd; zd;thetad; a1d; a2d];
gammadd = [xdd; zdd; thetadd; a1dd; a2dd];

% Variables for chainDiff function
vars = [gamma; gammad];
dvars = [gammad; gammadd];

% Sphere frame
uav.G = [uav_Gx; uav_Gy; uav_Gz];
uav.J = [uav_Jxx uav_Jxy uav_Jxz; ...
       uav_Jxy uav_Jyy uav_Jyz; ...
       uav_Jxz uav_Jyz uav_Jzz];
   
% Ballast frame 1
p1.G = [p1_Gx; p1_Gy; p1_Gz];
p1.J = [p1_Jxx p1_Jxy p1_Jxz; ...
       p1_Jxy p1_Jyy p1_Jyz; ...
       p1_Jxz p1_Jyz p1_Jzz];
   
   
% Ballast frame 2
p2.G = [p2_Gx; p2_Gy; p2_Gz];
p2.J = [p2_Jxx p2_Jxy p2_Jxz; ...
       p2_Jxy p2_Jyy p2_Jyz; ...
       p2_Jxz p2_Jyz p2_Jzz];   
   

% FK: I --> s --> p1 --> p1
   
% .R: position of the the frame from the previous frame it's attached to
% .T: transformation from frame to previous frame 

% Sphere frame
uav.R = [x; 0; z];  % I -> uav
uav.T = roty(theta); % I -> uav

syms theta0
%roty( theta0 )*rotx(pi/2)*rotz(a1)
% Ballast frame 1 
p1.R = [b_x; 0; 0]; % uav -> p1
p1.T = roty(a1); % uav -> p1


% Ballast frame 2)
p2.R = [b_x; 0; 0]; % p1 -> p2
p2.T = roty(a2); % p1 -> p2


% Define positions and velocities

IIRuav = uav.R;
% IIRsdot = jacobian(IIRs, gamma)*gammad;
IIRuavdot = chainDiff(IIRuav, vars, dvars);
ITuav = uav.T;
ITuavdot = chainDiff(ITuav, vars, dvars);

IIRp1 = IIRuav + ITuav*p1.T*p1.R;
IIRp1dot = chainDiff(IIRp1, vars, dvars);
ITp1 = ITuav*p1.T;
ITp1dot = chainDiff(ITp1, vars, dvars);

% for p2


IIRp2 = IIRp1 + ITp1*p2.T*p2.R;
IIRp2dot = chainDiff(IIRp2, vars, dvars);
ITp2 = ITp1*p2.T;
ITp2dot = chainDiff(ITp2, vars, dvars);




% Compute Kinetic and Potential Energies
uav.KE = KineticEnergy(IIRuavdot, uav_m, ITuavdot, uav.G, uav.J);
uav.PE = PotentialEnergy(IIRuav, uav_m, ITuav, uav.G, gz);

p1.KE = KineticEnergy(IIRp1dot, p1_m, ITp1dot, p1.G, p1.J);
p1.PE = PotentialEnergy(IIRp1, p1_m, ITp1, p1.G, gz);

p2.KE = KineticEnergy(IIRp2dot, p2_m, ITp2dot, p2.G, p2.J);
p2.PE = PotentialEnergy(IIRp2, p2_m, ITp2, p2.G, gz);


KE = uav.KE + p1.KE + p2.KE;
PE = uav.PE + p1.PE + p2.PE;

%Sk_w = simplify(ITsdot * ITs.');
%IswI = unskew(Sk_w);
%Iv = r*cross(IswI, [0;0;1]); % <-- Constraint equation

% Pfaffian matrix: multiples by gammad to get 0 for constraint
%A = jacobian(gammad(1:2,1) - Iv(1:2,1), gammad)
%Adot = chainDiff(A, vars, dvars)

% Compute Lagrangian of the system
L = KE - PE;

% Compute the system mass matrix
dL_dgammad = vectPartial(L,gammad);
dL_dgamma = vectPartial(L, gamma);

F = vectChainDiff(dL_dgammad, vars, dvars) - dL_dgamma;

H = jacobian(jacobian(KE, gammad).', gammad);
D = jacobian(jacobian(KE, gammad).', gamma)*gammad - jacobian(KE, gamma).';
G = jacobian(PE, gamma).';

%H = simplify(H)
%D = simplify(D)
%G = simplify(G)

%N = F-H*gammadd
q_vec = [x z theta a1 a2];
q_vec_dot = [xd zd thetad a1d a2d];


syms bad_info
N = ones(length(H))*bad_info;
for i = 1:length(H)
    for j = 1:length(H)
        temp =0;
        for k = 1:length(H)
            
            temp =  temp + (diff(H(j,k),q_vec(i))- diff(H(i,k),q_vec(j)))*q_vec_dot(k);
            
        end
        
        N(i,j) = temp;
        
        
    end
    
end


check = q_vec_dot*N*q_vec_dot.';

simplify(check)

% should be zero
H=simplify(H);
disp('found simple H')
D=simplify(D);
disp('found simple D')
G=simplify(G);
disp('found simple G')
DYNParamfile = 'HDGfile.mat';
save(DYNParamfile, 'H', 'D','G');
disp('Sys Param saved')
 

%load('HDGfile.mat')


