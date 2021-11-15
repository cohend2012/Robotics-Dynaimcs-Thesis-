function [ Xd,G ] = BallDynamics( t,X, F, params )

    Xd = zeros(14,1);

    % Split State Vector arguments
    x = X(1);
    y = X(2);
    phi = X(3);
    theta = X(4);
    psi = X(5);
    a1 = X(6);
    a2 = X(7);
    xd = X(8);
    yd = X(9);
    phid = X(10);
    thetad = X(11);
    psid = X(12);
    a1d = X(13);
    a2d = X(14);
    
    gamma = [x; y; phi; theta; psi; a1; a2];
    gammad = [xd; yd; phid; thetad; psid; a1d; a2d];
    
    % Split Physical Parameters Structure
    s_m = params.MassOfBody;
    b_m = params.MassOfBallast;
    r = params.SphereRadius;
    gz = params.AccelerationDueToGravity;
    
    s_Jxx = params.sphere.J(1,1);
    s_Jxy = params.sphere.J(1,2);
    s_Jxz = params.sphere.J(1,3);
    s_Jyy = params.sphere.J(2,2);
    s_Jyz = params.sphere.J(2,3);
    s_Jzz = params.sphere.J(3,3);
    
    b_Jxx = params.ballast.J(1,1);
    b_Jxy = params.ballast.J(1,2);
    b_Jxz = params.ballast.J(1,3);
    b_Jyy = params.ballast.J(2,2);
    b_Jyz = params.ballast.J(2,3);
    b_Jzz = params.ballast.J(3,3);
    
    s_Gx = params.sphere.G(1,1);
    s_Gy = params.sphere.G(2,1);
    s_Gz = params.sphere.G(3,1);
    
    b_Gx = params.ballast.G(1,1);
    b_Gy = params.ballast.G(2,1);
    b_Gz = params.ballast.G(3,1);
    
    b_x = params.ballast.pos(1,1);
    b_y = params.ballast.pos(2,1);
    b_z = params.ballast.pos(3,1);
    
    loadH;
    loadG;
    loadD;
    
    A = [ 1, 0, 0, -r*cos(phi), r*cos(theta)*sin(phi), 0, 0; ...
          0, 1, r,           0,          r*sin(theta), 0, 0];
      
    Adot = [ 0, 0, 0, phid*r*sin(phi), phid*r*cos(phi)*cos(theta) - r*thetad*sin(phi)*sin(theta), 0, 0; ...
             0, 0, 0,               0,                                       r*thetad*cos(theta), 0, 0];
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

