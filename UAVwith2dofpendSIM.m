%% DEFINE PARAMETERS

params.MassOfUAV = 2;
params.MassOfp1 = 100;
params.MassOfp2 = 100;

params.AccelerationDueToGravity = 9.79;

params.UAV.J = eye(3);
params.UAV.G = zeros(3,1);

params.p1.J = eye(3);
params.p1.G = [1;1;1];

params.p2.J = eye(3);
params.p2.G = [1;1;1];

params.UAV.pos = ones(3,1);

params.p1.pos = [1 ;0; 0];

params.p2.pos = [1 ;0; 0];


%% IMPLEMENT 4th ORDER RUNGA-KUTTA INTEGRATION

tSim = 1; % [s] Simulation Time
dt = 0.1; % [s] Simulation Time Step

t = 0:dt:tSim; % [s] simulation time

Fs = 20; % [Hz] Sample Rate/Processor Clock Speed (whichever is slower)
Ts = 1/Fs; % [s] Sampling interval
T_last = -Ts;

gamma = zeros(10, length(t));
gamma(2,1) = 10; % start with it above the ground. 
gamma(4,1) = pi/2;
gamma(5,1) = pi/2;
%gamma(9,1) = 10;
%gammaz(10,1) = 10;
err_ = zeros(2,1);
errInt_ = zeros(2,1);

F = zeros(5,1);
%(4) = 10;
F(5) = 10;
for ii = 1:length(t) - 1
    %if t(ii) - T_last >= Ts
        
        
        %gamma(:,11) = 4;
    %end  
    [k1,G] = UAVDynamics(t(ii),gamma(:,ii), F, params);
    [k2,G] = UAVDynamics(t(ii),gamma(:,ii)+k1*dt/2, F, params);
    [k3,G] = UAVDynamics(t(ii),gamma(:,ii)+k2*dt/2, F, params);
    [k4,G] = UAVDynamics(t(ii),gamma(:,ii)+k3*dt, F, params);
    gamma(:,ii+1) = gamma(:,ii) + dt*(k1(:,1)/6 + k2(:,1)/3 + k3(:,1)/3 + k4(:,1)/6)
end

% Plot Joint Variables
figure(1);

subplot(8,1,1);
plot(t, gamma(1,:), 'b-');
xlabel('t (s)');
ylabel('x (m)');

subplot(8,1,2);
plot(t, gamma(2,:), 'b-');
xlabel('t (s)');
ylabel('z (m)');

subplot(8,1,3);
plot(t, gamma(3,:), 'b-');
xlabel('t (s)');
ylabel('\theta (rad)');

subplot(8,1,4);
plot(t, gamma(4,:), 'b-');
xlabel('t (s)');
ylabel('\alpha_1 (rad)');

subplot(8,1,5);
plot(t, gamma(5,:), 'b-');
xlabel('t (s)');
ylabel('\alpha_2 (rad)');

subplot(8,1,6);
plot(t, gamma(6,:), 'b-');
xlabel('t (s)');
ylabel('x dot (rad)');

subplot(8,1,7);
plot(t, gamma(7,:), 'b-');
xlabel('t (s)');
ylabel('z dot (rad)');



hold on
% figure(2);
% 
% plot(t, G(4), 'b-');
% xlabel('t (s)');
% ylabel('G');


