figure(97);

color = 'r';

x = gamma_ARSR(1,:) - (gamma(1,1:10:end) + gamma(4,1:10:end));
y = gamma_ARSR(2,:) - (gamma(2,1:10:end) + gamma(5,1:10:end));
psi = gamma_ARSR(3,:) - (gamma(3,1:10:end) + gamma(6,1:10:end));
thMW = gamma_ARSR(4,:) - gamma(7,1:10:end);
th1 = gamma_ARSR(5,:) - gamma(8,1:10:end);
th2 = gamma_ARSR(6,:) - gamma(9,1:10:end);
th3 = gamma_ARSR(7,:) - gamma(10,1:10:end);
th4 = gamma_ARSR(8,:) - gamma(11,1:10:end);

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