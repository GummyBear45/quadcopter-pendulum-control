%% Live control: steer the drone's setpoint with the arrow keys while the model runs
%
%   Arrow keys  move the setpoint by `step` metres (left/right = x, up/down = y)
%   0 / Home    setpoint back to (0, 0)
%   A / D       push the mass left / right (or the Push buttons; strength menu)
%   Q / Esc     stop (closing the window also stops)
%
% A push is a force on the mass, perpendicular to the rod, lasting `pushDur`
% seconds of simulation time: the torque d = F*L goes into the tunable Constant
% block 'Live kick' (added to the disturbance) and is reset by the listener.
% The model runs in real time: the listener below waits whenever the simulation
% gets ahead of the wall clock (Simulink's own pacing stalls once we draw). The setpoint is the tunable
% Constant block 'Live setpoint'; every key press changes its value with
% set_param, and the Rate Limiter in the model turns it into a smooth ramp.
% A runtime listener on the 50 Hz 'Frame sampler' block redraws the scene as the simulation runs.

clear; close all; clc;
here = fileparts(mfilename('fullpath'));
cd(here);
mdl = 'pendulum';
if ~isfile(fullfile(here, [mdl '.slx'])), build_pendulum_model; end
load_system(mdl);

step    = 0.25;         % setpoint change per key press [m]
lim     = 3;            % setpoint limit (|x|, |y|) [m]
fps     = 25;           % redraw rate [Hz] (every 2nd 50 Hz frame; one redraw ~8 ms)
history = 15;           % seconds shown in the live plot
pushF   = [0.5 2 4];    % push presets [N]: nudge | max. that balance absorbs | knocks it over
pushNames = {'Small 0.5 N', 'Medium 2 N', 'Large 4 N'};
pushDur = 0.2;          % push duration [s] (simulation time)

pendulum_params;        % for drawing (L, m, l_arm, Tmax, ...)

%% Run settings: live setpoint, endless, paced to real time
initOrig = get_param(mdl, 'InitFcn');
% key presses write literal values into this block; always start from the variable
set_param([mdl '/Live setpoint'], 'Value', 'p_live');
set_param([mdl '/Live kick'], 'Value', 'kick_live');
set_param(mdl, 'InitFcn', [initOrig ' live_on = 1; p_live = [0; 0]; kick_live = 0; Tsim = inf;'], ...
               'EnablePacing', 'off');

%% Figure
S = struct();                              % shared state for the callbacks
S.mdl = mdl;  S.blk = [mdl '/Live setpoint'];
S.sp  = [0 0];  S.step = step;  S.lim = lim;
S.kblk = [mdl '/Live kick'];  S.pushF = pushF;  S.pushIdx = 2;  S.pushDur = pushDur;
S.kickOn = false;  S.kickPending = false;  S.kickEnd = -inf;  S.kickD = 0;  S.thLast = 0;
S.arrowHold = 0.4;                         % keep the push arrow visible this long after a push [s]
S.lastDraw = tic;  S.dtDraw = 1/fps;  S.tDraw = 0;  S.nDraw = 0;  S.roT = [];  S.roM = [];  S.wall0 = [];  S.simT0 = 0;

fig = figure('Name', 'Live pendulum (arrows: setpoint, A/D: push)', ...
             'Position', [150 100 1200 600], 'NumberTitle', 'off');
tl  = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
tl.OuterPosition = [0 0.1 1 0.9];          % room for the push controls

ax1 = nexttile(tl);
hold(ax1, 'on'); axis(ax1, 'equal'); grid(ax1, 'on'); box(ax1, 'on');
xlabel(ax1, 'x [m]'); ylabel(ax1, 'y [m]');
S.ax1    = ax1;
S.hPath  = animatedline(ax1, 'LineStyle', ':', 'Color', [0.30 0.75 0.93], 'MaximumNumPoints', 600);
S.hRef   = plot(ax1, 0, 0, '+', 'Color', [1 0.85 0.2], 'MarkerSize', 18, 'LineWidth', 2);
S.hRod   = plot(ax1, NaN, NaN, '-', 'Color', [0.55 0.55 0.6], 'LineWidth', 3);
S.hBody  = plot(ax1, NaN, NaN, '-', 'Color', [0.30 0.75 0.93], 'LineWidth', 5);
S.hRotor = plot(ax1, NaN, NaN, '-', 'Color', [0.80 0.80 0.85], 'LineWidth', 2);
S.hThr   = quiver(ax1, [0 0], [0 0], [0 0], [0 0], 0, 'Color', [1 0.85 0.2], ...
                  'LineWidth', 2, 'MaxHeadSize', 0.6);
S.hMass  = plot(ax1, NaN, NaN, 'o', 'MarkerSize', 10 + 20*m, 'MarkerEdgeColor', 'k', ...
                'MarkerFaceColor', [0.85 0.33 0.10]);
S.hPush  = quiver(ax1, 0, 0, 0, 0, 0, 'Color', [1 0.6 0], 'LineWidth', 3, ...
                  'MaxHeadSize', 2, 'Visible', 'off');
S.hPushT = text(ax1, 0, 0, '', 'Color', [1 0.6 0], 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center');
S.hTitle = title(ax1, 'starting...');
S.view   = 2.2;                            % half-width of the view around the setpoint [m]

ax2 = nexttile(tl);
hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
xlabel(ax2, 't [s]'); ylabel(ax2, '[m]');
title(ax2, 'Drone position vs setpoint');
np = history * fps;
S.lx  = animatedline(ax2, 'Color', [0.00 0.45 0.74], 'LineWidth', 1.5, 'MaximumNumPoints', np);
S.ly  = animatedline(ax2, 'Color', [0.85 0.33 0.10], 'LineWidth', 1.5, 'MaximumNumPoints', np);
S.lxr = animatedline(ax2, 'Color', [0.30 0.60 0.90], 'LineStyle', '--', 'MaximumNumPoints', np);
S.lyr = animatedline(ax2, 'Color', [0.95 0.55 0.30], 'LineStyle', '--', 'MaximumNumPoints', np);
legend(ax2, 'x', 'y', 'x_{set}', 'y_{set}', 'Location', 'northwest');
S.ax2 = ax2;  S.history = history;
% Push controls (their KeyPressFcn forwards keys, so arrows still work after a click)
uicontrol(fig, 'Style', 'pushbutton', 'String', '◀ Push', 'FontWeight', 'bold', ...
    'Units', 'normalized', 'Position', [0.02 0.045 0.08 0.045], ...
    'Callback', @(~, ~) doPush(fig, -1), 'KeyPressFcn', @(~, e) onKey(fig, e));
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Push ▶', 'FontWeight', 'bold', ...
    'Units', 'normalized', 'Position', [0.105 0.045 0.08 0.045], ...
    'Callback', @(~, ~) doPush(fig, +1), 'KeyPressFcn', @(~, e) onKey(fig, e));
uicontrol(fig, 'Style', 'popupmenu', 'String', pushNames, 'Value', 2, ...
    'Units', 'normalized', 'Position', [0.195 0.05 0.09 0.035], ...
    'Callback', @(src, ~) setPushIdx(fig, src.Value), 'KeyPressFcn', @(~, e) onKey(fig, e));
annotation(fig, 'textbox', [0.01 0.0 0.98 0.04], 'EdgeColor', 'none', ...
    'HorizontalAlignment', 'center', 'Color', [0.6 0.6 0.6], ...
    'String', ['Arrow keys: move setpoint   |   0 / Home: back to (0, 0)   |   ' ...
               'A / D: push left / right   |   Q / Esc: stop']);

S.L = L;  S.l_arm = l_arm;  S.Tmax = Tmax;
guidata(fig, S);
fig.KeyPressFcn     = @onKey;
fig.CloseRequestFcn = @onClose;

%% Start (non-blocking), restore InitFcn, attach the drawing listener
set_param(mdl, 'SimulationCommand', 'start');
t0 = tic;
while ~any(strcmp(get_param(mdl, 'SimulationStatus'), {'running', 'stopped'})) && toc(t0) < 30
    pause(0.05);
end
set_param(mdl, 'InitFcn', initOrig);        % InitFcn has run; leave the model as it was
if strcmp(get_param(mdl, 'SimulationStatus'), 'running')
    S = guidata(fig);
    % Listen to the 50 Hz Frame sampler, NOT to a continuous block: those fire on
    % every solver stage (~3000x per simulated second during swing-up), and that
    % callback overhead is what made the swing-up lag behind real time.
    S.listener = add_exec_event_listener([mdl '/Frame sampler'], 'PostOutputs', ...
                                         @(blk, ~) onStep(blk, fig));
    guidata(fig, S);
    figure(fig);                            % keyboard focus on the live window
else
    warning('live_pendulum: simulation did not start (status: %s).', ...
            get_param(mdl, 'SimulationStatus'));
end

%% ---------------------------------------------------------------- callbacks
function onKey(fig, evt)
S = guidata(fig);
switch evt.Key
    case 'leftarrow',  S.sp(1) = S.sp(1) - S.step;
    case 'rightarrow', S.sp(1) = S.sp(1) + S.step;
    case 'uparrow',    S.sp(2) = S.sp(2) + S.step;
    case 'downarrow',  S.sp(2) = S.sp(2) - S.step;
    case {'0', 'numpad0', 'home'}, S.sp = [0 0];
    case 'a', doPush(fig, -1); return
    case 'd', doPush(fig, +1); return
    case {'q', 'escape'}, onClose(fig); return
    otherwise, return
end
S.sp = min(max(S.sp, -S.lim), S.lim);
set_param(S.blk, 'Value', sprintf('[%g; %g]', S.sp(1), S.sp(2)));   % live tune
set(S.hRef, 'XData', S.sp(1), 'YData', S.sp(2));
guidata(fig, S);
end

function doPush(fig, dirX)
% Push the mass horizontally towards dirX (-1 left, +1 right): a force F
% perpendicular to the rod, i.e. torque d = F*L, with the sign that gives the
% mass a velocity component in dirX (tangent t = (cos th, sin th), so sign(cos th)).
S = guidata(fig);
F = S.pushF(S.pushIdx);
s = sign(cos(S.thLast));  if s == 0, s = 1; end
S.kickD = dirX * s * F * S.L;
set_param(S.kblk, 'Value', sprintf('%g', S.kickD));   % starts at the next 20 ms sample
S.kickOn = true;  S.kickPending = true;               % listener stamps the end time
guidata(fig, S);
end

function setPushIdx(fig, k)
S = guidata(fig);  S.pushIdx = k;  guidata(fig, S);
end

function onStep(blk, fig)
if ~isvalid(fig), return; end
S = guidata(fig);
t = blk.CurrentTime;
% push timing in simulation time: stamp the end on the first frame, then switch off
if S.kickPending
    S.kickEnd = t + S.pushDur;  S.kickPending = false;  guidata(fig, S);
elseif S.kickOn && t >= S.kickEnd
    set_param(S.kblk, 'Value', '0');
    S.kickOn = false;  guidata(fig, S);
end
if isempty(S.wall0), S.wall0 = tic; S.simT0 = t; guidata(fig, S); end
lead = (t - S.simT0) - toc(S.wall0);                   % sim ahead of wall clock?
if lead < -0.1                                         % fell behind (MATLAB was busy):
    S.wall0 = tic; S.simT0 = t; guidata(fig, S);       % drop the backlog, never fast-forward
    lead = 0;
end
if lead > 0.015, pause(lead); end                      % real-time pacing (+ key events); coarse
                                                       % chunks: each pause costs a ~15 ms timer tick
if toc(S.lastDraw) < S.dtDraw, return; end            % throttle to ~fps
S = guidata(fig);                                      % keys may have changed it
S.lastDraw = tic;
tDraw0 = tic;
x  = blk.OutputPort(1).Data;               % sampled [theta omega x y vx vy beta betadot]
th = x(1); px = x(3); py = x(4); be = x(7);
S.thLast = th;
if isempty(S.roT)                          % cache the runtime objects (get_param is slow)
    S.roT = get_param([S.mdl '/Motor thrust'], 'RuntimeObject');
    S.roM = get_param([S.mdl '/Controller/Catch-release (hysteresis)'], 'RuntimeObject');
end
T  = S.roT.OutputPort(1).Data;
md = S.roM.OutputPort(1).Data;

e = [cos(be); sin(be)];  n = [-sin(be); cos(be)];  c0 = [px; py];
rL = c0 - S.l_arm*e + 0.05*n;  rR = c0 + S.l_arm*e + 0.05*n;  rb = 0.1;
xm = px + S.L*sin(th);  ym = py - S.L*cos(th);
set(S.hRod,  'XData', [px xm], 'YData', [py ym]);
set(S.hMass, 'XData', xm, 'YData', ym);
set(S.hBody, 'XData', px + [-1 1]*S.l_arm*e(1), 'YData', py + [-1 1]*S.l_arm*e(2));
set(S.hRotor, 'XData', [rL(1)-rb*e(1), rL(1)+rb*e(1), NaN, rR(1)-rb*e(1), rR(1)+rb*e(1)], ...
              'YData', [rL(2)-rb*e(2), rL(2)+rb*e(2), NaN, rR(2)-rb*e(2), rR(2)+rb*e(2)]);
len = T(:).' * 0.5 * S.L / S.Tmax;
set(S.hThr, 'XData', [rL(1) rR(1)], 'YData', [rL(2) rR(2)], 'UData', n(1)*len, 'VData', n(2)*len);
addpoints(S.hPath, px, py);
if S.kickOn || S.kickPending || t < S.kickEnd + S.arrowHold   % push arrow (held a little longer)
    tv  = sign(S.kickD) * [cos(th), sin(th)];
    len = 0.15 + 0.12*abs(S.kickD)/S.L;
    set(S.hPush, 'XData', xm - tv(1)*len, 'YData', ym - tv(2)*len, ...
                 'UData', tv(1)*len, 'VData', tv(2)*len, 'Visible', 'on');
    set(S.hPushT, 'Position', [xm - tv(1)*len, ym - tv(2)*len + 0.15], ...
                  'String', sprintf('%.1f N', abs(S.kickD)/S.L));
else
    S.hPush.Visible = 'off';  S.hPushT.String = '';
end
if md == 1
    S.hMass.MarkerFaceColor = [0.47 0.67 0.19]; modeStr = 'BALANCE';
else
    S.hMass.MarkerFaceColor = [0.85 0.33 0.10]; modeStr = 'SWING-UP';
end
S.hTitle.String = sprintf('%s   t = %5.1f s   set (%.2f, %.2f)   drone (%.2f, %.2f)', ...
                          modeStr, t, S.sp(1), S.sp(2), px, py);
% keep the view on the setpoint and the drone
cx = (S.sp(1) + px)/2;  cy = (S.sp(2) + py)/2;
axis(S.ax1, [cx - S.view, cx + S.view, cy - S.view*0.8, cy + S.view*0.8]);

addpoints(S.lx, t, px);  addpoints(S.ly, t, py);
addpoints(S.lxr, t, S.sp(1));  addpoints(S.lyr, t, S.sp(2));
xlim(S.ax2, [max(0, t - S.history), max(S.history, t)]);
drawnow limitrate;
S.tDraw = S.tDraw + toc(tDraw0);  S.nDraw = S.nDraw + 1;   % redraw cost (diagnostics)
guidata(fig, S);
end

function onClose(fig, ~)
S = guidata(fig);
if isfield(S, 'listener') && isvalid(S.listener), delete(S.listener); end
if any(strcmp(get_param(S.mdl, 'SimulationStatus'), {'running', 'paused'}))
    set_param(S.mdl, 'SimulationCommand', 'stop');
end
set_param(S.blk, 'Value', 'p_live');        % undo the live literals, leave the model clean
set_param(S.kblk, 'Value', 'kick_live');
set_param(S.mdl, 'Dirty', 'off');           % everything is restored -> no "save changes?" prompt
delete(fig);
end
