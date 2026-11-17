%% Runs the Simulink pendulum model, plots the state vector and animates it
clear; close all; clc;

here = fileparts(mfilename('fullpath'));
cd(here);
mdl = 'pendulum';

if ~isfile(fullfile(here, [mdl '.slx']))
    build_pendulum_model;
end

pendulum_params;                 % same values the model uses (for plotting)
out = sim(mdl);

% --- Extract state vector ------------------------------------------------
t     = out.x.Time;
X     = squeeze(out.x.Data);
if size(X, 1) ~= numel(t)
    X = X.';                     % make it N x 8:  [theta omega x y vx vy beta betadot]
end
theta = X(:, 1);
omega = X(:, 2);
px    = X(:, 3);   py = X(:, 4);                         % drone (pivot) position
beta  = X(:, 7);                                         % drone tilt
Tr    = squeeze(out.T.Data);
if size(Tr, 1) ~= numel(out.T.Time), Tr = Tr.'; end
Tr    = interp1(out.T.Time, Tr, t);                      % rotor-pair thrusts [T1 T2]
bdes  = interp1(out.beta_des.Time, squeeze(out.beta_des.Data), t);
Pref  = interp1(out.p_ref.Time, out.p_ref.Data, t, 'previous');      % waypoint reference [x y]
Aa    = squeeze(out.a.Data);
if size(Aa, 1) == 2 && size(Aa, 2) ~= 2, Aa = Aa.'; end
ax_   = interp1(out.a.Time, Aa(:, 1), t);                % drone acceleration
ay_   = interp1(out.a.Time, Aa(:, 2), t);
md    = interp1(out.mode.Time, double(squeeze(out.mode.Data)), t, 'previous');  % 1 = balance

% Times at which the controller switches between swing-up and balance
iSw     = find(diff(md) ~= 0) + 1;
tSwitch = t(iSw);
swLabel = repmat({'lost'}, size(iSw));
swLabel(md(iSw) == 1) = {'catch'};            % entering balance mode
markSwitches = @() arrayfun(@(k) xline(tSwitch(k), '--', swLabel{k}, 'LineWidth', 1, ...
                    'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off'), ...
                    1:numel(tSwitch));

% Disturbance torque on the same grid, and shaded windows where it acts
dt_   = interp1(out.d.Time, squeeze(out.d.Data), t, 'previous');
iOn   = find(dist_on * dist_amp ~= 0);
markDist = @() arrayfun(@(i) xregion(dist_t(i), dist_t(i) + dist_dur(i), ...
                    'FaceColor', [1 0.6 0], 'FaceAlpha', 0.35, 'HandleVisibility', 'off'), iOn);

% Energy (without control it can only decrease due to damping)
Ek = 0.5 * m * L^2 * omega.^2;
Ep = m * g * L * (1 - cos(theta));
E  = Ek + Ep;

%% State vector plots
figure('Name', 'Pendulum state vector', 'Position', [100 30 1100 1000]);
tiledlayout(5, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t, rad2deg(theta), 'LineWidth', 1.5);
if ctrl_on
    yline(rad2deg(theta_ref) * [-1 1], '--', 'upright', 'LineWidth', 1);
    markSwitches();
end
markDist();
grid on; xlabel('t [s]'); ylabel('\theta [deg]');
title('Angle \theta(t)   (orange = disturbance)');

nexttile;
plot(t, omega, 'LineWidth', 1.5, 'Color', [0.85 0.33 0.10]);
if ctrl_on, markSwitches(); end
markDist();
grid on; xlabel('t [s]'); ylabel('\omega [rad/s]');
title('Angular velocity \omega(t)');

nexttile;
plot(t, px, 'LineWidth', 1.5, 'Color', [0.00 0.45 0.74]); hold on;
plot(t, py, 'LineWidth', 1.5, 'Color', [0.85 0.33 0.10]);
plot(t, Pref(:, 1), '--', 'LineWidth', 1.1, 'Color', [0.30 0.60 0.90]);
plot(t, Pref(:, 2), '--', 'LineWidth', 1.1, 'Color', [0.95 0.55 0.30]);
if ctrl_on, markSwitches(); end
markDist();
grid on; xlabel('t [s]'); ylabel('[m]');
title('Drone position vs waypoint reference');
legend('x', 'y', 'x_{ref}', 'y_{ref}', 'Location', 'best');

nexttile;
plot(px, py, 'LineWidth', 1.2, 'Color', [0.30 0.75 0.93]);
hold on; plot(Pref(:, 1), Pref(:, 2), '--', 'LineWidth', 1, 'Color', [0.6 0.6 0.6]);
plot(waypoints(:, 2), waypoints(:, 3), 'kd', 'MarkerFaceColor', [1 0.85 0.2], 'MarkerSize', 8);
plot(px(1), py(1), 'go', 'MarkerFaceColor', 'g');
plot(px(end), py(end), 'rs', 'MarkerFaceColor', 'r');
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]');
title('Drone path'); legend('path', 'reference', 'waypoints', 'start', 'end', 'Location', 'best');

nexttile;
plot(rad2deg(theta), omega, 'LineWidth', 1.2);
hold on; plot(rad2deg(theta(1)), omega(1), 'go', 'MarkerFaceColor', 'g');
plot(rad2deg(theta(end)), omega(end), 'rs', 'MarkerFaceColor', 'r');
grid on; xlabel('\theta [deg]'); ylabel('\omega [rad/s]');
title('Phase portrait'); legend('trajectory', 'start', 'end', 'Location', 'best');

nexttile;
plot(t, Ek, t, Ep, t, E, 'LineWidth', 1.3);
if ctrl_on
    yline(E_up, '--', 'E_{up} = 2mgL', 'LineWidth', 1, 'HandleVisibility', 'off');
end
markDist();
grid on; xlabel('t [s]'); ylabel('E [J]');
title('Energy'); legend('kinetic', 'potential', 'total', 'Location', 'best');

nexttile([1 2]);
yyaxis left
plot(t, ax_, '-', 'LineWidth', 1.4, 'Color', [0.00 0.45 0.74]); hold on;
plot(t, ay_, '-', 'LineWidth', 1.4, 'Color', [0.47 0.67 0.19]);
if ctrl_on, markSwitches(); end
ylabel('commanded a_{des} [m/s^2]');
yyaxis right
plot(t, dt_, '-', 'LineWidth', 1, 'Color', [1 0.6 0]);
ylabel('disturbance d [N\cdotm]');
grid on; xlabel('t [s]');
legend('a_x', 'a_y', 'disturbance d', 'Location', 'southwest');
title(sprintf('Outer loop   swing-up: k_E = %g   balance: K_b = [%.1f  %.1f  %.2f  %.2f]   |a| \\leq %g', ...
              kE, Kb, amax));

nexttile;
plot(t, rad2deg(bdes), '--', 'LineWidth', 1.1, 'Color', [0.6 0.6 0.6]); hold on;
plot(t, rad2deg(beta), 'LineWidth', 1.4, 'Color', [0.30 0.75 0.93]);
if ctrl_on, markSwitches(); end
grid on; xlabel('t [s]'); ylabel('\beta [deg]');
title(sprintf('Drone tilt (attitude loop \\omega_n = %g rad/s)', wn_att));
legend('\beta_{des}', '\beta', 'Location', 'best');

nexttile;
plot(t, Tr(:, 1), 'LineWidth', 1.2, 'Color', [0.85 0.33 0.10]); hold on;
plot(t, Tr(:, 2), 'LineWidth', 1.2, 'Color', [0.47 0.67 0.19]);
yline([0 Tmax], ':', 'LineWidth', 1, 'HandleVisibility', 'off');
yline(T_hover, '--', 'hover', 'LineWidth', 1, 'HandleVisibility', 'off');
if ctrl_on, markSwitches(); end
grid on; xlabel('t [s]'); ylabel('thrust [N]');
ylim([-1 Tmax + 2]);
title(sprintf('Rotor-pair thrusts (\\tau_m = %g s)', tau_m));
legend('T_1 left', 'T_2 right', 'Location', 'best');

%% Animation (interactive player: Play/Pause, Replay, speed, time slider)
% Space = play/pause, R = replay, +/- = speed. Runs on a timer, so the
% command window stays free; close the window to stop it.
animate_pendulum(t, theta, omega, md, L, m, ctrl_on, dt_, px, py, beta, Tr, l_arm, Tmax, Pref);
