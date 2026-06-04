%% Índice de manipulabilidad — actualizado a resolución fina
N = length(theta_2_tray);   % 101 puntos automáticamente
w = zeros(1, N);

for i = 1:N
    w(i) = abs(L_1 * L_2 * L_3 * sin(theta_2_tray(i)));
end

figure;
plot(tsim, w, 'r-', 'LineWidth', 3, 'DisplayName', '\mu');
hold on;
yline(0, 'w--', 'LineWidth', 1.5, 'DisplayName', 'Singularidad');
hold off;
legend
title('Índice de manipulabilidad — \mu = |L_1 L_2 L_3 \sin(\theta_2)|')
xlabel('t [s]')
ylabel('\mu')
grid on;