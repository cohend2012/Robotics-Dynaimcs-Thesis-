function [ Xd,G ] = UAVDynamics( t,X, F, params )

    Xd = zeros(15,1);

    % Split State Vector arguments
    x = X(1);
    y = X(2);
    z = X(2);
    phi = X(4);
    theta = X(5);
    psi = X(6);
    a1 = X(7);
    a2 = X(8);
    xd = X(9);
    yd = X(10);
    phid = X(11);
    thetad = X(12);
    psid = X(13);
    a1d = X(14);
    a2d = X(15);
    
    gamma = [x; y; z; phi; theta; psi; a1; a2];
    gammad = [xd; yd;zd; phid; thetad; psid; a1d; a2d];
    
    % Split Physical Parameters Structure
    UAV_m = params.MassOfUAV;
    p1_m = params.MassOfp1;
    p2 = params.MassOfp2;
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
    
    
    p1_x = params.p1.pos(1,1);
    p1_y = params.p1.pos(2,1);
    p1_z = params.p1.pos(3,1);
    
    
    p2_x = params.p2.pos(1,1);
    p2_y = params.p2.pos(2,1);
    p2_z = params.p2.pos(3,1);
    
    
    
    
    load('HDGfile.mat')
    
    
    subs(H,X,params)
    
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
    
    tmp = [H A'; A zeros(2,2)]\[F - D - G; -Adot*gammad];
    
    Xd(1:7,1) = X(8:14,1);
    
    Xd(8:14,1) = tmp(1:7,1);
 

end

