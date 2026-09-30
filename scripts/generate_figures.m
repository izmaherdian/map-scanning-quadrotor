%% GENERATE_FIGURES  Export the result figures used in the README to docs/images/results.
% Loads the saved simulation results in data/scanning_mode_results.mat and writes
% static PNG plots plus an animated GIF of the 3D trajectory tracking.
%
% Usage (from the repository root):
%   matlab -batch "run('scripts/generate_figures.m')"

close all

projectRoot = fullfile(fileparts(mfilename('fullpath')), '..');
S = load(fullfile(projectRoot, 'data', 'scanning_mode_results.mat'));
outDir = fullfile(projectRoot, 'docs', 'images', 'results');
if ~exist(outDir, 'dir'), mkdir(outDir); end

% Colors: actual = blue (solid), desired = orange (dashed)
cAct = [42 120 214]/255;
cDes = [235 104 52]/255;
cInk = [0.25 0.25 0.25];

t = S.t;
% NED frame -> plotting frame (y to the left, z up)
toPlot = @(P) [P(:,1), -P(:,2), -P(:,3)];
Pa = toPlot([S.x S.y S.z]);
Pd = toPlot([S.x_d S.y_d S.z_d]);
EA = [S.phi S.theta S.psi];
ED = [S.phi_d S.theta_d S.psi_d];
U  = S.simOut.U_out.Data;

%% RMSE (same definition as main_simulation)
rmsePos = sqrt(mean(([S.x S.y S.z] - [S.x_d S.y_d S.z_d]).^2, 1));
rmseAtt = sqrt(mean((EA - ED).^2, 1));
fprintf('RMSE x = %.4f m, y = %.4f m, z = %.4f m\n', rmsePos);
fprintf('RMSE phi = %.6f rad, theta = %.6f rad, psi = %.6g rad\n', rmseAtt);

%% 1. 3D trajectory
fig = newFig(900, 700);
plot3(Pd(:,1), Pd(:,2), Pd(:,3), '--', 'Color', cDes, 'LineWidth', 1.5); hold on
plot3(Pa(:,1), Pa(:,2), Pa(:,3), '-', 'Color', cAct, 'LineWidth', 2);
plot3(Pa(1,1), Pa(1,2), Pa(1,3), 'o', 'MarkerSize', 8, 'MarkerFaceColor', cInk, 'MarkerEdgeColor', 'w');
grid on; axis equal; view(-50, 28)
xlabel('x (m)'); ylabel('-y (m)'); zlabel('altitude -z (m)')
title('3D trajectory tracking (zig-zag scanning mode)')
legend({'Desired', 'Actual', 'Start'}, 'Location', 'northeast')
styleAxes(gca)
save_png(fig, outDir, 'trajectory_3d.png')

%% 2. Top view (xy plane)
fig = newFig(800, 700);
plot(Pd(:,1), Pd(:,2), '--', 'Color', cDes, 'LineWidth', 1.5); hold on
plot(Pa(:,1), Pa(:,2), '-', 'Color', cAct, 'LineWidth', 2);
grid on; axis equal
xlabel('x (m)'); ylabel('-y (m)')
title('Scanning path, top view (xy plane)')
legend({'Desired', 'Actual'}, 'Location', 'bestoutside')
styleAxes(gca)
save_png(fig, outDir, 'trajectory_xy.png')

%% 3. Position tracking
labels = {'x (m)', '-y (m)', 'altitude -z (m)'};
fig = newFig(1000, 750);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:3
    nexttile
    plot(t, Pd(:,k), '--', 'Color', cDes, 'LineWidth', 1.5); hold on
    plot(t, Pa(:,k), '-', 'Color', cAct, 'LineWidth', 2);
    ylabel(labels{k}); grid on; xlim([t(1) t(end)]); styleAxes(gca)
    if k == 1, legend({'Desired', 'Actual'}, 'Location', 'northeast'); end
end
xlabel(tl, 'Time (s)'); title(tl, 'Position tracking')
save_png(fig, outDir, 'position_tracking.png')

%% 4. Attitude tracking
labels = {'\phi roll (rad)', '\theta pitch (rad)', '\psi yaw (rad)'};
fig = newFig(1000, 750);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:3
    nexttile
    plot(t, ED(:,k), '--', 'Color', cDes, 'LineWidth', 1.5); hold on
    plot(t, EA(:,k), '-', 'Color', cAct, 'LineWidth', 1.5);
    ylabel(labels{k}); grid on; xlim([t(1) t(end)]); styleAxes(gca)
    if k == 1, legend({'Desired', 'Actual'}, 'Location', 'northeast'); end
end
xlabel(tl, 'Time (s)'); title(tl, 'Attitude tracking (Euler angles)')
save_png(fig, outDir, 'attitude_tracking.png')

%% 5. Position tracking error
labels = {'e_x (m)', 'e_y (m)', 'e_z (m)'};
E = [S.x_d S.y_d S.z_d] - [S.x S.y S.z];
fig = newFig(1000, 650);
tl = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:3
    nexttile
    plot(t, E(:,k), '-', 'Color', cAct, 'LineWidth', 1.5);
    yline(0, ':', 'Color', cInk);
    ylabel(labels{k}); grid on; xlim([t(1) t(end)]); styleAxes(gca)
    text(0.99, 0.9, sprintf('RMSE = %.4f m', rmsePos(k)), 'Units', 'normalized', ...
        'HorizontalAlignment', 'right', 'Color', cInk, 'FontSize', 10)
end
xlabel(tl, 'Time (s)'); title(tl, 'Position tracking error (desired - actual)')
save_png(fig, outDir, 'position_error.png')

%% 6. Control inputs
labels = {'U_1 (thrust)', 'U_2 (roll)', 'U_3 (pitch)', 'U_4 (yaw)'};
fig = newFig(1000, 800);
tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for k = 1:4
    nexttile
    plot(t, U(:,k), '-', 'Color', cAct, 'LineWidth', 1.2);
    ylabel(labels{k}); grid on; xlim([t(1) t(end)]); styleAxes(gca)
end
xlabel(tl, 'Time (s)'); title(tl, 'Control inputs')
save_png(fig, outDir, 'control_inputs.png')

%% 7. Animated GIF of the 3D tracking
gifFile = fullfile(outDir, 'tracking_animation.gif');
fig = newFig(720, 560);
ax = axes(fig);
plot3(ax, Pd(:,1), Pd(:,2), Pd(:,3), '--', 'Color', [cDes 0.6], 'LineWidth', 1.2); hold(ax, 'on')
trail = plot3(ax, nan, nan, nan, '-', 'Color', cAct, 'LineWidth', 2);
arms = gobjects(2, 1);
for a = 1:2, arms(a) = plot3(ax, nan(1,2), nan(1,2), nan(1,2), 'k-', 'LineWidth', 2.5); end
target = plot3(ax, nan, nan, nan, 'o', 'MarkerSize', 6, 'MarkerFaceColor', cDes, 'MarkerEdgeColor', 'w');
grid(ax, 'on'); axis(ax, 'equal'); view(ax, -50, 28)
pad = 5;
xlim(ax, [min(Pd(:,1))-pad max(Pd(:,1))+pad]);
ylim(ax, [min(Pd(:,2))-pad max(Pd(:,2))+pad]);
zlim(ax, [0 max(Pd(:,3))+pad]);
xlabel(ax, 'x (m)'); ylabel(ax, '-y (m)'); zlabel(ax, 'altitude (m)')
legend(ax, {'Desired', 'Actual'}, 'Location', 'northeast', 'AutoUpdate', 'off')
styleAxes(ax)
armLen = 3;   % drawn arm length (m), exaggerated for visibility
frameTimes = 0:1:t(end);
for f = 1:numel(frameTimes)
    [~, i] = min(abs(t - frameTimes(f)));
    set(trail, 'XData', Pa(1:i,1), 'YData', Pa(1:i,2), 'ZData', Pa(1:i,3));
    set(target, 'XData', Pd(i,1), 'YData', Pd(i,2), 'ZData', Pd(i,3));
    R = diag([1 -1 -1]) * body2inertial(EA(i,1), EA(i,2), EA(i,3));
    c = Pa(i,:);
    for a = 1:2
        dirv = armLen * (R(:,1) + (3 - 2*a) * R(:,2))';   % "X" configuration
        P = [c - dirv; c + dirv];
        set(arms(a), 'XData', P(:,1), 'YData', P(:,2), 'ZData', P(:,3));
    end
    title(ax, sprintf('Quadrotor zig-zag scanning   t = %5.1f s', t(i)))
    drawnow
    [im, map] = rgb2ind(frame2im(getframe(fig)), 128, 'nodither');
    if f == 1
        imwrite(im, map, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', 0.08);
    else
        imwrite(im, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.08);
    end
end
close(fig)
fprintf('Figures written to %s\n', outDir);

%% Helpers
function fig = newFig(w, h)
    fig = figure('Color', 'w', 'Position', [50 50 w h], 'Visible', 'off');
end

function styleAxes(ax)
    set(ax, 'FontSize', 11, 'GridAlpha', 0.15, 'XColor', [0.3 0.3 0.3], ...
        'YColor', [0.3 0.3 0.3], 'ZColor', [0.3 0.3 0.3], 'Box', 'off');
end

function save_png(fig, outDir, name)
    exportgraphics(fig, fullfile(outDir, name), 'Resolution', 150);
    close(fig)
end

function m = body2inertial(phi, theta, psi)
    % Same ZYX rotation as scripts/animate_quadrotor_3d.m (matrixB2I)
    R_ItoV1  = [cos(psi) sin(psi) 0; -sin(psi) cos(psi) 0; 0 0 1];
    R_V1toV2 = [cos(theta) 0 -sin(theta); 0 1 0; sin(theta) 0 cos(theta)];
    R_V2toB  = [1 0 0; 0 cos(phi) sin(phi); 0 -sin(phi) cos(phi)];
    m = (R_V2toB * R_V1toV2 * R_ItoV1)';
end
