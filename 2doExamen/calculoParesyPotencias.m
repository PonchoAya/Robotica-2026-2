%% Pares y Potencias — Dinámica del robot SCARA
% Usa los ángulos, velocidades y aceleraciones del espacio de trabajo

%% Verificación de dependencias
if ~exist('theta_1_tray','var') || ~exist('theta_1_v','var') || ~exist('theta_1_a','var')
    error('Ejecuta primero los scripts de trayectoria, velocidad y aceleración.')
end

tf   = 10;
dt = 0.1;
tsim = 0:dt:tf;

N = length(tsim);

tao_1_c   = zeros(1, N);
tao_2_c   = zeros(1, N);
tao_3_c   = zeros(1, N);
pot_1     = zeros(1, N);
pot_2     = zeros(1, N);
pot_3     = zeros(1, N);
pot_total = zeros(1, N);

for i = 1:N

    % Ángulos — del espacio de trabajo (cinemática inversa punto a punto)
    theta_1   = theta_1_tray(i);
    theta_2   = theta_2_tray(i);
    theta_3   = theta_3_tray(i);

    % Velocidades — del espacio de trabajo (Jacobiano inverso)
    theta_1_vc = theta_1_v(i);
    theta_2_vc = theta_2_v(i);
    theta_3_vc = theta_3_v(i);

    % Aceleraciones — del espacio de trabajo (Jacobiano inverso con J_dot)
    theta_1_ac = theta_1_a(i);
    theta_2_ac = theta_2_a(i);
    theta_3_ac = theta_3_a(i);

    % Términos sigma — expresiones trigonométricas intermedias
    sigma_19 = sin(theta_2 + theta_1) / 2;
    sigma_18 = sin(theta_1 + theta_2 + theta_3);
    sigma_17 = cos(theta_2 + theta_1) / 2;
    sigma_16 = cos(theta_1 + theta_2 + theta_3);
    sigma_15 = theta_2_vc * (sigma_18/8 + sigma_19);
    sigma_14 = sigma_18/8 + sigma_19 + sin(theta_1)/2;
    sigma_13 = theta_2_vc * (sigma_16/8 + sigma_17);
    sigma_12 = sigma_16/8 + sigma_17 + cos(theta_1)/2;
    sigma_11 = theta_3_vc*sigma_18/8 + theta_1_vc*sigma_14 + sigma_15;
    sigma_10 = sigma_13 + theta_3_vc*sigma_16/8 + theta_1_vc*sigma_12;
    sigma_9  = ((sigma_16/8 + sigma_17) * sigma_11) / 2;
    sigma_8  = ((sigma_18/8 + sigma_19) * sigma_10) / 2;
    sigma_7  = (sigma_16*(sigma_16/8 + sigma_17))/16 + (sigma_18*(sigma_18/8 + sigma_19))/16 + 1;
    sigma_6  = sigma_16*sigma_12/16 + sigma_18*sigma_14 + 1;
    sigma_5  = theta_2_vc*sigma_16/8 + theta_3_vc*sigma_16/8 + theta_1_vc*sigma_16/8;
    sigma_4  = theta_2_vc*sigma_18/8 + theta_3_vc*sigma_18/8 + theta_1_vc*sigma_18/8;
    sigma_3  = cos(theta_2)/8 + ((sigma_16/8 + sigma_17)*sigma_12)/2 + ((sigma_18/8 + sigma_19)*sigma_14)/2 + 33/16;
    sigma_2  = sigma_13 + theta_1_vc*(sigma_16/8 + sigma_17) + theta_3_vc*sigma_16/8;
    sigma_1  = theta_3_vc*sigma_18/8 + sigma_15 + theta_1_vc*(sigma_18/8 + sigma_19);

    % Par de la articulación 1 (base)
    tao_1_c(i) = theta_1_ac*(cos(theta_2)/4 + sigma_12^2/2 + sigma_14^2/2 + 27/8) ...
               - theta_3_vc*(sigma_18*sigma_10/16 - sigma_16*sigma_11/16 + sigma_12*sigma_4/2 - sigma_5*sigma_14/2) ...
               + theta_2_ac*sigma_3 ...
               + theta_3_ac*sigma_6 ...
               - theta_2_vc*(sigma_8 - sigma_14*sigma_2/2 + sigma_1*sigma_12/2 - sigma_9 + theta_2_vc*sin(theta_2)/8 + theta_1_vc*sin(theta_2)/4);

    % Par de la articulación 2 (codo)
    tao_2_c(i) = theta_1_vc^2*sin(theta_2)/8 ...
               - theta_3_vc*(sigma_18*sigma_10 - ((sigma_18/8 + sigma_19)*sigma_5)/2 + ((sigma_16/8 + sigma_17)*sigma_4)/2 - sigma_16*sigma_11/16) ...
               + theta_2_ac*sigma_7 ...
               + sigma_10*sigma_1/2 ...
               + theta_1_ac*sigma_3 ...
               - sigma_11*sigma_2/2 ...
               + theta_2_ac*(((sigma_16 + sigma_17)^2)/2 + ((sigma_18 + sigma_19)^2)/2 + 33/16) ...
               - theta_2_vc*((sigma_16/8 + sigma_17)*sigma_1/2 + (sigma_18/8 + sigma_19)*sigma_2/2 + sigma_8 - sigma_9 + theta_1_vc*sin(theta_2)/8) ...
               + theta_2_vc*theta_1_vc*sin(theta_2)/8;

    % Par de la articulación 3 (muñeca)
    tao_3_c(i) = theta_3_ac*(sigma_16^2/128 + sigma_18^2/128 + 1) ...
               - sigma_5*sigma_11/2 ...
               + theta_2_ac*sigma_7 ...
               - theta_3_vc*(sigma_18*sigma_10/16 - sigma_18*sigma_5/16 - sigma_16*sigma_11/16 + sigma_16*sigma_4/16) ...
               + theta_2_vc*(sigma_18*sigma_2/16 - sigma_18*sigma_10/16 - sigma_16*sigma_1/16 + sigma_16*sigma_11/16) ...
               + theta_1_ac*sigma_6 ...
               + sigma_10*sigma_4/2;

    % Potencias instantáneas — P = |τ · dθ/dt|
    pot_1(i)     = abs(tao_1_c(i) * theta_1_vc);
    pot_2(i)     = abs(tao_2_c(i) * theta_2_vc);
    pot_3(i)     = abs(tao_3_c(i) * theta_3_vc);
    pot_total(i) = pot_1(i) + pot_2(i) + pot_3(i);
end

%% Resumen numérico
fprintf('\n=== Pares máximos ===\n')
[val, idx] = max(abs(tao_1_c)); fprintf('τ1 máx = %.4f N·m  (t = %.1f s)\n', val, tsim(idx))
[val, idx] = max(abs(tao_2_c)); fprintf('τ2 máx = %.4f N·m  (t = %.1f s)\n', val, tsim(idx))
[val, idx] = max(abs(tao_3_c)); fprintf('τ3 máx = %.4f N·m  (t = %.1f s)\n', val, tsim(idx))
fprintf('\n=== Potencias máximas ===\n')
[val, idx] = max(pot_1);     fprintf('P1 máx    = %.4f W  (t = %.1f s)\n', val, tsim(idx))
[val, idx] = max(pot_2);     fprintf('P2 máx    = %.4f W  (t = %.1f s)\n', val, tsim(idx))
[val, idx] = max(pot_3);     fprintf('P3 máx    = %.4f W  (t = %.1f s)\n', val, tsim(idx))
[val, idx] = max(pot_total); fprintf('P total máx = %.4f W  (t = %.1f s)\n', val, tsim(idx))

%% Gráfica de pares
figure;
plot(tsim, tao_1_c, 'r-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\tau_1');
hold on;
plot(tsim, tao_2_c, 'g-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\tau_2');
plot(tsim, tao_3_c, 'b-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', '\tau_3');
hold off;
legend
title('Pares de las articulaciones — Dinámica del robot SCARA')
xlabel('t [s]')
ylabel('\tau [N·m]')
grid on;

%% Gráfica de potencias
figure;
plot(tsim, pot_1,     'r-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', 'P_1');
hold on;
plot(tsim, pot_2,     'g-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', 'P_2');
plot(tsim, pot_3,     'b-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', 'P_3');
plot(tsim, pot_total, 'w-o', 'LineWidth', 3, 'MarkerSize', 6, 'DisplayName', 'P_{total}');
hold off;
legend
title('Potencias de las articulaciones — Dinámica del robot SCARA')
xlabel('t [s]')
ylabel('P [W]')
grid on;