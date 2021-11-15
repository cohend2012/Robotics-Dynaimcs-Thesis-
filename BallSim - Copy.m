%% DEFINE PARAMETERS

params.MassOfBody = 3;
params.MassOfBallast = 5;
params.SphereRadius = 0.4064;
params.AccelerationDueToGravity = 9.79;

params.sphere.J = eye(3);
params.sphere.G = zeros(3,1);
params.ballast.J = eye(3);
params.ballast.G = [0;0;-1];

params.ballast.pos = zeros(3,1);

%% IMPLEMENT 4th ORDER RUNGA-KUTTA INTEGRATION

tSim = 5; % [s] Simulation Time
dt = 0.01; % [s] Simulation Time Step

t = 0:dt:tSim; % [s] simulation time

Fs = 20; % [Hz] Sample Rate/Processor Clock Speed (whichever is slower)
Ts = 1/Fs; % [s] Sampling interval
T_last = -Ts;

gamma = zeros(14, length(t));

%gamma(7,1) = pi/2;

err_ = zeros(2,1);
errInt_ = zeros(2,1);

F = zeros(7,1);

for ii = 1:length(t) - 1
    if t(ii) - T_last >= Ts
        
        F(6) = 1;
        
    end  
    [k1,G] = BallDynamics(t(ii),gamma(:,ii), F, params);
    [k2,G] = BallDynamics(t(ii),gamma(:,ii)+k1*dt/2, F, params);
    [k3,G] = BallDynamics(t(ii),gamma(:,ii)+k2*dt/2, F, params);
    [k4,G] = BallDynamics(t(ii),gamma(:,ii)+k3*dt, F, params);
    gamma(:,ii+1) = gamma(:,ii) + dt*(k1(:,1)/6 + k2(:,1)/3 + k3(:,1)/3 + k4(:,1)/6);
end

% Plot Joint Variables
figure(1);

subplot(7,1,1);
plot(t, gamma(1,:), 'b-');
xlabel('t (s)');
ylabel('x (m)');

subplot(7,1,2);
plot(t, gamma(2,:), 'b-');
xlabel('t (s)');
ylabel('y (m)');

subplot(7,1,3);
plot(t, gamma(3,:), 'b-');
xlabel('t (s)');
ylabel('\phi (rad)');

subplot(7,1,4);
plot(t, gamma(4,:), 'b-');
xlabel('t (s)');
ylabel('\theta (rad)');

subplot(7,1,5);
plot(t, gamma(5,:), 'b-');
xlabel('t (s)');
ylabel('\psi (rad)');

subplot(7,1,6);
plot(t, gamma(6,:), 'b-');
xlabel('t (s)');
ylabel('\alpha_1 (rad)');

subplot(7,1,7);
plot(t, gamma(7,:), 'b-');
xlabel('t (s)');
ylabel('\alpha_2 (rad)');
hold on
figure(2);

plot(t, G(4), 'b-');
xlabel('t (s)');
ylabel('G');


% Animate Ball
% AnimateBall;