function [ Xd,G ] = UAVDynamics( t,X, F, params )

    Xd = zeros(10,1);

    % Split State Vector arguments
    x = X(1);
    z = X(2);
    theta = X(3);
    a1 = X(4);
    a2 = X(5);
    
    xd = X(6);
    zd = X(7);
    thetad = X(8);
    a1d = X(9);
    a2d = X(10);
    
    %gamma = [x; y; z; phi; theta; psi; a1; a2];
    %gammad = [xd; yd;zd; phid; thetad; psid; a1d; a2d];
    
    % Split Physical Parameters Structure
    UAV_m = params.MassOfUAV;
    p1_m = params.MassOfp1;
    p2_m = params.MassOfp2;
    %r = params.SphereRadius;
    gz = params.AccelerationDueToGravity;
    

    UAV_Jxx = params.UAV.J(1,1);
    UAV_Jxy = params.UAV.J(1,2);
    UAV_Jxz = params.UAV.J(1,3);
    UAV_Jyy = params.UAV.J(2,2);
    UAV_Jyz = params.UAV.J(2,3);
    UAV_Jzz = params.UAV.J(3,3);
    
    p1_Jxx = params.p1.J(1,1);
    p1_Jxy = params.p1.J(1,2);
    p1_Jxz = params.p1.J(1,3);
    p1_Jyy = params.p1.J(2,2);
    p1_Jyz = params.p1.J(2,3);
    p1_Jzz = params.p1.J(3,3);
    

    p2_Jxx = params.p2.J(1,1);
    p2_Jxy = params.p2.J(1,2);
    p2_Jxz = params.p2.J(1,3);
    p2_Jyy = params.p2.J(2,2);
    p2_Jyz = params.p2.J(2,3);
    p2_Jzz = params.p2.J(3,3);
    
    
    
    
    UAV_Gx = params.UAV.G(1,1);
    UAV_Gy = params.UAV.G(2,1);
    UAV_Gz = params.UAV.G(3,1);
    
    p1_Gx = params.p1.G(1,1);
    p1_Gy = params.p1.G(2,1);
    p1_Gz = params.p1.G(3,1);
    
    p2_Gx = params.p2.G(1,1);
    p2_Gy = params.p2.G(2,1);
    p2_Gz = params.p2.G(3,1);
    
    
        
    UAV_x = params.UAV.pos(1,1);
    UAV_y = params.UAV.pos(2,1);
    UAV_z = params.UAV.pos(3,1);
    
    
    
    
    p1_x = params.p1.pos(1,1);
    p1_y = params.p1.pos(2,1);
    p1_z = params.p1.pos(3,1);
    
   
    
    p2_x = params.p2.pos(1,1);
    p2_y = params.p2.pos(2,1);
    p2_z = params.p2.pos(3,1);
    
    b_x = p2_x;
    b_y = 0;
    
Sym_vars = {'x';'z' ; 'theta';'a1' ; 'a2'  ;'xd'  ;'zd'; 'thetad' ;'a1d'; 'a2d' ;...
                'uav_m' ;'p1_m' ;'p2_m' ;'gz' ;'b_x' ;'b_y';'uav_Jxx'; 'uav_Jyy' ;'uav_Jzz'; 'uav_Jxy'; 'uav_Jxz' ;'uav_Jyz'; ...
                'p1_Jxx' ;'p1_Jyy' ;'p1_Jzz'; 'p1_Jxy' ;'p1_Jxz'; 'p1_Jyz'; ...
                'p2_Jxx'; 'p2_Jyy' ;'p2_Jzz' ;'p2_Jxy' ;'p2_Jxz' ;'p2_Jyz';...
                'uav_Gx' ;'uav_Gy' ;'uav_Gz' ;'p1_Gx' ;'p1_Gy' ;'p1_Gz';...
                'p2_Gx'; 'p2_Gy'; 'p2_Gz'};
            
            
    real_val_var =  [x ;z ;theta; a1 ; a2  ;xd; zd ; thetad ;a1d; a2d ;...
                UAV_m ;p1_m ;p2_m ;gz ;b_x ;b_y;UAV_Jxx; UAV_Jyy ;UAV_Jzz; UAV_Jxy; UAV_Jxz ;UAV_Jyz; ...
                p1_Jxx ;p1_Jyy ;p1_Jzz; p1_Jxy ;p1_Jxz; p1_Jyz; ...
                p2_Jxx; p2_Jyy ;p2_Jzz ;p2_Jxy ;p2_Jxz ;p2_Jyz;...
                UAV_Gx ;UAV_Gy ;UAV_Gz ;p1_Gx ;p1_Gy ;p1_Gz;...
                p2_Gx; p2_Gy; p2_Gz];
            
            
    
            
            
%syms x y z phi theta psi a1 a2
%syms xd yd zd phid thetad psid a1d a2d
%syms xdd ydd zdd phidd thetadd psidd a1dd a2dd

%syms uav_m b_m p1_m p2_m r d gz b_x b_y b_z

%syms uav_Jxx uav_Jyy uav_Jzz uav_Jxy uav_Jxz uav_Jyz % UAV: inertia matrix, center of the sphere
%syms p1_Jxx p1_Jyy p1_Jzz p1_Jxy p1_Jxz p1_Jyz % Pend1: inertia matrix, center of the pendulum joint
%syms p2_Jxx p2_Jyy p2_Jzz p2_Jxy p2_Jxz p2_Jyz % Pend2: inertia matrix, center of the pendulum joint


%syms uav_Gx uav_Gy uav_Gz % UAV: vector of first mass moments
%syms p1_Gx p1_Gy p1_Gz % Pend1: vector of first mass moments
%syms p2_Gx p2_Gy p2_Gz % Pend1: vector of first mass moments          
            
            
    
    load('HDGfile.mat')
    
    
    H = subs(H,Sym_vars,real_val_var);
    D = subs(D,Sym_vars,real_val_var);
    G = subs(G,Sym_vars,real_val_var);
    
    %A = [ 1, 0, 0, -r*cos(phi), r*cos(theta)*sin(phi), 0, 0; ...
    %      0, 1, r,           0,          r*sin(theta), 0, 0];
      
   % Adot = [ 0, 0, 0, phid*r*sin(phi), phid*r*cos(phi)*cos(theta) - r*thetad*sin(phi)*sin(theta), 0, 0; ...
   %          0, 0, 0,               0,                                       r*thetad*cos(theta), 0, 0];
     %disp(H)
     %disp(G(3))
    
%    hold on 
%    figure(1);
%    subplot(7,1,1);
%    plot(t, G(3), 'b-');
%    xlabel('t (s)');
%    ylabel('G');
    
    %tmp = [H A'; A zeros(2,2)]\[F - D - G; -Adot*gammad];
    
    
    
    sol = H\(F-D-G);
    disp('found sol')
    Xd(1:5,1) = X(6:10,1);
    Xd(6:10) = sol(1:5);
 

end

