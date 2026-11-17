function fig = animate_pendulum(t, theta, omega, md, L, m, ctrl_on, d, px, py, beta, T, l_arm, Tmax, pref)
%ANIMATE_PENDULUM Interactive replay of a pendulum simulation.
%   fig = animate_pendulum(t, theta, omega, md, L, m, ctrl_on, d, px, py, beta, T, l_arm, Tmax, pref)
%
%   pref (optional, N x 2): the drone position reference; drawn as a moving target.
%
%   px, py (optional) are the pivot (drone) positions; a drone is drawn there.
%   beta, T, l_arm, Tmax (optional): drone tilt, rotor-pair thrusts [T1 T2]
%   (N x 2), arm length and thrust limit; the body is drawn tilted and each
%   rotor pair gets a thrust arrow proportional to T/Tmax.
%
%   d (optional) is the disturbance torque on the same time grid; while it
%   is non-zero an orange arrow shows the push on the mass.
%
%   Controls:  Play/Pause button  (or Space)
%              Replay button      (or R)
%              Speed menu         (or +/- keys)
%              Time slider        drag to scrub through the motion
%
%   Runs on a MATLAB timer, so the command window stays usable and the
%   figure can be replayed as often as you like. Closing the figure stops it.

if nargin < 8 || isempty(d), d = zeros(size(t)); end
if nargin < 10, px = zeros(size(t)); py = zeros(size(t)); end   % fixed pivot
if nargin < 11 || isempty(beta), beta = zeros(size(t)); end
if nargin < 12 || isempty(T),    T = nan(numel(t), 2);  end     % no thrust arrows
if nargin < 13 || isempty(l_arm), l_arm = 0.35 * L; end
if nargin < 14 || isempty(Tmax),  Tmax = 1; end
if nargin < 15 || isempty(pref),  pref = zeros(numel(t), 2); end

speeds    = [0.1 0.25 0.5 1 2];
speedIdx  = 3;              % start at 0.5x so the beginning is easy to follow
tNow      = 0;
playing   = true;
lastTick  = [];
tmr       = [];
tEnd      = t(end);

% Uniform grid for fast frame lookup
fpsGrid = 100;
tA  = (0:1/fpsGrid:tEnd).';
thA = interp1(t, theta, tA);
omA = interp1(t, omega, tA);
mdA = interp1(t, md, tA, 'previous');
dA  = interp1(t, d,  tA, 'previous');
arrowScale = 0.5 * L / max([abs(d(:)); 1]);   % largest push -> arrow of L/2
pxA = interp1(t, px, tA);
pyA = interp1(t, py, tA);
beA = interp1(t, beta, tA);
TA  = interp1(t, T, tA);
PrA = interp1(t, pref, tA, 'previous');
xA  = pxA + L * sin(thA);
yA  = pyA - L * cos(thA);

delete(timerfindall('Tag', 'pendulumAnim'));   % leftovers from earlier runs

%% Figure and axes
fig = figure('Name', 'Pendulum animation', 'Position', [250 120 1000 560], ...
             'KeyPressFcn', @onKey, 'CloseRequestFcn', @onClose);
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
tl.OuterPosition = [0 0.1 1 0.9];             % leave room for the controls

ax1 = nexttile(tl);
hold(ax1, 'on'); axis(ax1, 'equal'); grid(ax1, 'on'); box(ax1, 'on');
pad = 1.3 * L;
xl  = [min([px; pref(:,1)]) - pad, max([px; pref(:,1)]) + pad];
yl  = [min([py; pref(:,2)]) - pad, max([py; pref(:,2)]) + pad];
axis(ax1, [xl yl]);
hRef = plot(ax1, NaN, NaN, '+', 'Color', [1 0.85 0.2], 'MarkerSize', 16, 'LineWidth', 1.5);  % reference
hDPath = plot(ax1, NaN, NaN, ':', 'Color', [0.30 0.75 0.93]);          % drone path
hTrace = plot(ax1, NaN, NaN, '-', 'Color', [0.6 0.75 1]);
hRod   = plot(ax1, NaN, NaN, '-', 'Color', [0.55 0.55 0.6], 'LineWidth', 3);
% Drone: tilted body + two rotor pairs + thrust arrows (drawn around the pivot)
dw = l_arm;
hBody  = plot(ax1, NaN, NaN, '-', 'Color', [0.30 0.75 0.93], 'LineWidth', 5);
hRotor = plot(ax1, NaN, NaN, '-', 'Color', [0.80 0.80 0.85], 'LineWidth', 2);
hThr   = quiver(ax1, [0 0], [0 0], [0 0], [0 0], 0, 'Color', [1.0 0.85 0.2], ...
                'LineWidth', 2, 'MaxHeadSize', 0.6);
thrScale = 0.5 * L / Tmax;                       % full thrust -> arrow of L/2
hMass  = plot(ax1, NaN, NaN, 'o', 'MarkerSize', 10 + 10*m, 'MarkerEdgeColor', 'k');
hPush  = quiver(ax1, 0, 0, 0, 0, 0, 'Color', [1 0.6 0], 'LineWidth', 3, ...
                'MaxHeadSize', 2, 'Visible', 'off');
hPushT = text(ax1, 0, 0, '', 'Color', [1 0.6 0], 'FontWeight', 'bold', ...
              'HorizontalAlignment', 'center');
hTitle = title(ax1, '');
xlabel(ax1, 'x [m]'); ylabel(ax1, 'y [m]');

ax2 = nexttile(tl);
hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
xlim(ax2, [0 tEnd]);
ylim(ax2, [min([theta; omega]) max([theta; omega])] * 1.1 + [-0.1 0.1]);
% faint full trajectory in the background, bright part up to the current time
plot(ax2, t, theta, ':', 'Color', [0.30 0.45 0.70], 'HandleVisibility', 'off');
plot(ax2, t, omega, ':', 'Color', [0.70 0.40 0.25], 'HandleVisibility', 'off');
hTh  = plot(ax2, NaN, NaN, 'LineWidth', 1.5, 'Color', [0.00 0.45 0.74]);
hOm  = plot(ax2, NaN, NaN, 'LineWidth', 1.5, 'Color', [0.85 0.33 0.10]);
hNow = xline(ax2, 0, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
xlabel(ax2, 't [s]'); title(ax2, 'State vector x = [\theta; \omega]');
legend(ax2, '\theta [rad]', '\omega [rad/s]', 'Location', 'northeast');

%% Controls
hPlay = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Pause', ...
    'Units', 'normalized', 'Position', [0.02 0.02 0.08 0.05], 'Callback', @onPlay);
uicontrol(fig, 'Style', 'pushbutton', 'String', 'Replay', ...
    'Units', 'normalized', 'Position', [0.11 0.02 0.08 0.05], 'Callback', @onReplay);
uicontrol(fig, 'Style', 'text', 'String', 'Speed', ...
    'Units', 'normalized', 'Position', [0.20 0.02 0.05 0.04]);
hSpeed = uicontrol(fig, 'Style', 'popupmenu', ...
    'String', compose('%gx', speeds), 'Value', speedIdx, ...
    'Units', 'normalized', 'Position', [0.25 0.025 0.07 0.04], 'Callback', @onSpeed);
hSlider = uicontrol(fig, 'Style', 'slider', 'Min', 0, 'Max', tEnd, 'Value', 0, ...
    'SliderStep', [0.1 1] / tEnd, ...
    'Units', 'normalized', 'Position', [0.34 0.025 0.56 0.04]);
addlistener(hSlider, 'ContinuousValueChange', @onScrub);
hTimeLbl = uicontrol(fig, 'Style', 'text', 'String', '', ...
    'Units', 'normalized', 'Position', [0.905 0.02 0.08 0.04]);

drawFrame();
drawnow;                                       % render fully before the clock starts

%% Timer
tmr = timer('Tag', 'pendulumAnim', 'ExecutionMode', 'fixedSpacing', ...
            'Period', 0.03, 'BusyMode', 'drop', 'StartDelay', 1.0, ...
            'TimerFcn', @onTick);
start(tmr);

%% ---------------------------------------------------------------- nested
    function onTick(~, ~)
        if ~isvalid(fig), return; end
        if isempty(lastTick), lastTick = tic; end
        dtWall   = min(toc(lastTick), 0.1);   % a rendering hiccup must not skip motion
        lastTick = tic;
        if playing
            tNow = tNow + dtWall * speeds(speedIdx);
            if tNow >= tEnd
                tNow = tEnd;
                setPlaying(false);
            end
            drawFrame();
        end
    end

    function drawFrame()
        k = min(numel(tA), 1 + round(tNow * fpsGrid));
        set(hRod,  'XData', [pxA(k) xA(k)], 'YData', [pyA(k) yA(k)]);
        % body axes: e = along the arms, n = thrust direction (body up)
        e  = [cos(beA(k)); sin(beA(k))];
        n  = [-sin(beA(k)); cos(beA(k))];
        c0 = [pxA(k); pyA(k)];
        rL = c0 - dw*e + 0.05*n;   rR = c0 + dw*e + 0.05*n;    % rotor hubs
        set(hBody, 'XData', c0(1) + [-dw dw]*e(1), 'YData', c0(2) + [-dw dw]*e(2));
        rb = 0.1;                                              % rotor half-width
        set(hRotor, 'XData', [rL(1) - rb*e(1), rL(1) + rb*e(1), NaN, rR(1) - rb*e(1), rR(1) + rb*e(1)], ...
                    'YData', [rL(2) - rb*e(2), rL(2) + rb*e(2), NaN, rR(2) - rb*e(2), rR(2) + rb*e(2)]);
        if all(isfinite(TA(k, :)))
            len = TA(k, :) * thrScale;
            set(hThr, 'XData', [rL(1) rR(1)], 'YData', [rL(2) rR(2)], ...
                      'UData', n(1)*len, 'VData', n(2)*len, 'Visible', 'on');
        else
            hThr.Visible = 'off';
        end
        set(hDPath, 'XData', pxA(1:k), 'YData', pyA(1:k));
        set(hRef, 'XData', PrA(k, 1), 'YData', PrA(k, 2));
        set(hMass, 'XData', xA(k),     'YData', yA(k));
        i0 = max(1, k - round(1.5 * fpsGrid));          % 1.5 s trail
        set(hTrace, 'XData', xA(i0:k), 'YData', yA(i0:k));
        set(hTh, 'XData', tA(1:k), 'YData', thA(1:k));
        set(hOm, 'XData', tA(1:k), 'YData', omA(1:k));
        hNow.Value = tA(k);

        % Disturbance arrow: tangent to the circle, in the direction of +theta for d > 0
        if dA(k) ~= 0
            len = dA(k) * arrowScale;
            tx  = cos(thA(k));  ty = sin(thA(k));
            set(hPush, 'XData', xA(k) - tx*len, 'YData', yA(k) - ty*len, ...
                       'UData', tx*len, 'VData', ty*len, 'Visible', 'on');
            set(hPushT, 'Position', [xA(k) - 1.3*tx*len, yA(k) - 1.3*ty*len + 0.12], ...
                        'String', sprintf('d = %+.1f N·m', dA(k)));
        else
            hPush.Visible = 'off';
            hPushT.String = '';
        end

        if ~ctrl_on
            modeStr = 'FREE';     col = [0.85 0.33 0.10];
        elseif mdA(k) == 1
            modeStr = 'BALANCE';  col = [0.47 0.67 0.19];
        else
            modeStr = 'SWING-UP'; col = [0.85 0.33 0.10];
        end
        hMass.MarkerFaceColor = col;
        if all(isfinite(TA(k, :)))
            hTitle.String = sprintf('%s   \\theta = %6.1f^\\circ   \\beta = %5.1f^\\circ   T = %4.1f / %4.1f N', ...
                                    modeStr, rad2deg(thA(k)), rad2deg(beA(k)), TA(k, 1), TA(k, 2));
        else
            hTitle.String = sprintf('%s   \\theta = %6.1f^\\circ   \\omega = %5.2f rad/s', ...
                                    modeStr, rad2deg(thA(k)), omA(k));
        end
        hSlider.Value    = min(tA(k), tEnd);
        hTimeLbl.String  = sprintf('%.2f / %.1f s', tA(k), tEnd);
        drawnow limitrate;
    end

    function setPlaying(p)
        playing  = p;
        lastTick = tic;
        if p, hPlay.String = 'Pause'; else, hPlay.String = 'Play'; end
    end

    function onPlay(~, ~)
        if ~playing && tNow >= tEnd, tNow = 0; end     % Play at the end = replay
        setPlaying(~playing);
    end

    function onReplay(~, ~)
        tNow = 0;
        drawFrame();
        setPlaying(true);
    end

    function onSpeed(src, ~)
        speedIdx = src.Value;
    end

    function onScrub(src, ~)
        setPlaying(false);
        tNow = src.Value;
        drawFrame();
    end

    function onKey(~, evt)
        switch evt.Key
            case 'space'
                onPlay();
            case 'r'
                onReplay();
            case {'add', 'equal'}
                speedIdx = min(numel(speeds), speedIdx + 1); hSpeed.Value = speedIdx;
            case {'subtract', 'hyphen'}
                speedIdx = max(1, speedIdx - 1);             hSpeed.Value = speedIdx;
        end
    end

    function onClose(~, ~)
        if ~isempty(tmr) && isvalid(tmr), stop(tmr); delete(tmr); end
        delete(fig);
    end
end
