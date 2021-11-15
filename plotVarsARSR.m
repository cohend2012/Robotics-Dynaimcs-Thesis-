figure(98);

color = 'b';

x = gamma(1,:);
y = gamma(2,:);
psi = gamma(3,:);
thMW = gamma(4,:);
th1 = gamma(5,:);
th2 = gamma(6,:);
th3 = gamma(7,:);
th4 = gamma(8,:);

gamma_ARSR = gamma;

subplot(4,2,1);
plot(t, x, color); hold on; grid on;
xlabel('t (s)'); ylabel('x (m)');

subplot(4,2,3);
plot(t, y, color); hold on; grid on;
xlabel('t (s)'); ylabel('y (m)');

subplot(4,2,5);
plot(t, psi, color); hold on; grid on;
xlabel('t (s)'); ylabel('\psi (rad)');

subplot(4,2,7);
plot(t, thMW, color); hold on; grid on;
xlabel('t (s)'); ylabel('\theta_{MW} (rad)');

subplot(4,2,2);
plot(t, th1, color); hold on; grid on;
xlabel('t (s)'); ylabel('\theta_1 (rad)');

subplot(4,2,4);
plot(t, th2, color); hold on; grid on;
xlabel('t (s)'); ylabel('\theta_2 (rad)');

subplot(4,2,6);
plot(t, th3, color); hold on; grid on;
xlabel('t (s)'); ylabel('\theta_3 (rad)');

subplot(4,2,8);
plot(t, th4, color); hold on; grid on;
xlabel('t (s)'); ylabel('\theta_4 (rad)');