%% Builds pendulum.slx - inverted pendulum on a planar quadcopter
%
% Drone: mass M, inertia J, tilt beta, two rotor pairs (T1 at -l_arm, T2 at +l_arm)
% with first-order motor lag. Pendulum (point mass m, rod L) pinned at the drone
% centre. Coupled dynamics (Lagrange, q = [x; y; theta]):
%   (M+m)x'' + mL(cos th th'' - sin th th'^2) = -F sin(beta)
%   (M+m)y'' + mL(sin th th'' + cos th th'^2) = F cos(beta) - (M+m)g
%   mL^2 th'' + mL(cos th x'' + sin th y'')    = -mgL sin th - b(th' - beta') + d
%   J beta''                                   = l_arm(T2 - T1) + b(th' - beta')
%
% State vector: x = [theta; omega; x; y; vx; vy; beta; betadot]

mdl = 'pendulum';
here = fileparts(mfilename('fullpath'));

if bdIsLoaded(mdl)
    close_system(mdl, 0);
end
if isfile(fullfile(here, [mdl '.slx']))
    delete(fullfile(here, [mdl '.slx']));
end

new_system(mdl);
open_system(mdl);

set_param(mdl, ...
    'InitFcn',   'pendulum_params;', ...
    'StopTime',  'Tsim', ...
    'Solver',    'ode45', ...
    'RelTol',    '1e-8', ...
    'AbsTol',    '1e-10', ...
    'MaxStep',   '0.01', ...
    'ReturnWorkspaceOutputs', 'on', ...
    'BlockReduction', 'off');   % keep 'Frame sampler' (unused output) for the live listener

%% ---------------------------------------------------------------- plant
% Coupled drone + pendulum rigid-body dynamics (Lagrange, coordinates x, y, theta, beta)
%   input  xs = [theta; omega; x; y; vx; vy; beta; betadot],  T = [T1; T2],  d
%   output theta'', p'' = [x''; y''], beta''
addMLFcn([mdl '/Drone + pendulum dynamics'], [180 120 400 300], { ...
    'function [thdd, pdd, bdd] = f(xs, T, d, M, m, J, L, g, b, l_arm)'
    '% Generalised coordinates q = [x; y; theta]; the tilt beta is decoupled'
    '% because the rod is pinned at the drone centre of mass.'
    'th = xs(1); om = xs(2); be = xs(7); bd = xs(8);'
    'c = cos(th); s = sin(th);'
    'F   = T(1) + T(2);              % total thrust along the body axis'
    'tau = l_arm*(T(2) - T(1));      % right pair - left pair -> tilt torque'
    'Mt  = M + m;'
    'fr  = b*(om - bd);              % pivot friction (relative rotation)'
    'MM  = [Mt 0 m*L*c; 0 Mt m*L*s; m*L*c m*L*s m*L^2];'
    'rhs = [-F*sin(be) + m*L*s*om^2;'
    '        F*cos(be) - Mt*g - m*L*c*om^2;'
    '       -m*g*L*s - fr + d];'
    'qdd  = MM \ rhs;'
    'pdd  = qdd(1:2);'
    'thdd = qdd(3);'
    'bdd  = (tau + fr)/J;'
    }, {'M', 'm', 'J', 'L', 'g', 'b', 'l_arm'});

add_block('simulink/Continuous/Integrator', [mdl '/Integrator omega'], ...
    'InitialCondition', 'omega0', 'Position', [460 110 500 150]);
add_block('simulink/Continuous/Integrator', [mdl '/Integrator theta'], ...
    'InitialCondition', 'theta0', 'Position', [560 110 600 150]);
add_block('simulink/Continuous/Integrator', [mdl '/Integrator v'], ...
    'InitialCondition', '[0; 0]', 'Position', [460 190 500 230], ...
    'BackgroundColor', 'cyan');
add_block('simulink/Continuous/Integrator', [mdl '/Integrator p'], ...
    'InitialCondition', '[x0; y0]', 'Position', [560 190 600 230], ...
    'BackgroundColor', 'cyan');
add_block('simulink/Continuous/Integrator', [mdl '/Integrator betadot'], ...
    'InitialCondition', '0', 'Position', [460 270 500 310], ...
    'BackgroundColor', 'cyan');
add_block('simulink/Continuous/Integrator', [mdl '/Integrator beta'], ...
    'InitialCondition', 'beta0', 'Position', [560 270 600 310], ...
    'BackgroundColor', 'cyan');

% Full state vector xs (used by the plant and the inner loop)
add_block('simulink/Signal Routing/Mux', [mdl '/Mux state'], 'Inputs', '6', ...
    'DisplayOption', 'bar', 'Position', [660 90 665 330]);

%% ---------------------------------------------------------------- inner loop
% 1) Inverse dynamics: thrust vector F so that the drone really gets a_des
addMLFcn([mdl '/Thrust vector (inverse dynamics)'], [880 480 1080 540], { ...
    'function Fv = f(a, xs, M, m, L, g, b, tau_ff)'
    '% Predict theta'''' for the commanded pivot acceleration a (d unknown -> 0),'
    '% then the mass acceleration, then the force the rod pulls on the drone.'
    '% Evaluated tau_ff ahead: the tilt + motors need that long to deliver it,'
    '% and a late cancellation of the rod pull acts as damping on the swing.'
    'th = xs(1); om = xs(2); bd = xs(8);'
    'thdd0 = (-m*L*(cos(th)*a(1) + sin(th)*a(2)) - m*g*L*sin(th) - b*(om - bd)) / (m*L^2);'
    'th = th + tau_ff*om + 0.5*tau_ff^2*thdd0;'
    'om = om + tau_ff*thdd0;'
    'c = cos(th); s = sin(th);'
    'thdd = (-m*L*(c*a(1) + s*a(2)) - m*g*L*s - b*(om - bd)) / (m*L^2);'
    'qdd  = a + L*(thdd*[c; s] + om^2*[-s; c]);   % mass acceleration'
    'Fv   = M*(a + [0; g]) + m*(qdd + [0; g]);     % drone + what the rod needs'
    }, {'M', 'm', 'L', 'g', 'b', 'tau_ff'});

% 2) Attitude control + mixer: tilt the thrust onto Fv, split into T1 / T2
addMLFcn([mdl '/Attitude control + mixer'], [1120 480 1320 560], { ...
    'function [Tc, beta_des] = f(Fv, xs, kb_att, kbd_att, l_arm, Tmax, beta_max)'
    'be = xs(7); bd = xs(8);'
    'beta_des = atan2(-Fv(1), Fv(2));                 % body axis (-sin b, cos b) || Fv'
    'beta_des = min(max(beta_des, -beta_max), beta_max);'
    'e   = atan2(sin(be - beta_des), cos(be - beta_des));'
    'tau = -kb_att*e - kbd_att*bd;                    % attitude PD'
    '% Altitude priority: choose F so the VERTICAL force is right even while the'
    '% tilt is still lagging (a projection onto the axis makes the drone bob,'
    '% and that bobbing pumps energy out of the pendulum).'
    'F   = max(0, Fv(2) / max(cos(be), 0.5));'
    '% Attitude has priority: keep both pairs within [0, Tmax] by limiting F'
    'dT  = min(abs(tau)/(2*l_arm), Tmax/2);'
    'F   = min(max(F, 2*dT), 2*(Tmax - dT));'
    'tau = sign(tau)*2*l_arm*dT;'
    'Tc  = [F/2 - tau/(2*l_arm); F/2 + tau/(2*l_arm)];  % [left pair; right pair]'
    }, {'kb_att', 'kbd_att', 'l_arm', 'Tmax', 'beta_max'});

% 3) Motors: first-order thrust lag, T' = (Tc - T)/tau_m
add_block('simulink/Math Operations/Sum', [mdl '/Motor error'], ...
    'Inputs', '+-', 'IconShape', 'round', 'Position', [1370 505 1400 535]);
add_block('simulink/Math Operations/Gain', [mdl '/1//tau_m'], 'Gain', '1/tau_m', ...
    'Position', [1420 500 1470 540]);
add_block('simulink/Continuous/Integrator', [mdl '/Motor thrust'], ...
    'InitialCondition', '[T_hover; T_hover]', 'Position', [1500 500 1540 540], ...
    'BackgroundColor', 'yellow');

%% ---------------------------------------------------------------- disturbance
add_block('simulink/Sources/From Workspace', [mdl '/Disturbance'], ...
    'VariableName', 'dist_data', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Setting to zero', 'ZeroCross', 'on', ...
    'Position', [20 420 130 450], 'BackgroundColor', 'orange');
add_block('simulink/Sources/Random Number', [mdl '/Noise'], ...
    'Mean', '0', 'Variance', 'noise_std^2', 'Seed', 'noise_seed', ...
    'SampleTime', 'noise_Ts', ...
    'Position', [20 480 130 510], 'BackgroundColor', 'orange');
add_block('simulink/Math Operations/Sum', [mdl '/pulses + noise'], ...
    'Inputs', '+++', 'IconShape', 'round', 'Position', [160 450 190 480]);
% Live push (live_pendulum.m buttons): tunable torque, normally 0
add_block('simulink/Sources/Constant', [mdl '/Live kick'], ...
    'Value', 'kick_live', 'SampleTime', '0.02', ...
    'Position', [20 540 130 570], 'BackgroundColor', 'orange');

%% ---------------------------------------------------------------- reference
% Scripted waypoints [t x y] (held between entries), smoothed by a rate limiter
add_block('simulink/Sources/From Workspace', [mdl '/Waypoints'], ...
    'VariableName', 'wp_data', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', 'ZeroCross', 'on', ...
    'Position', [440 560 550 590], 'BackgroundColor', 'green');
% Live setpoint (tunable while running, see live_pendulum.m) or scripted waypoints
add_block('simulink/Sources/Constant', [mdl '/Live setpoint'], ...
    'Value', 'p_live', 'SampleTime', '0.02', ...   % periodic -> picks up live changes
    'Position', [440 500 550 530], 'BackgroundColor', 'green');
add_block('simulink/Sources/Constant', [mdl '/live_on'], ...
    'Value', 'live_on', 'Position', [440 530 480 550]);
add_block('simulink/Signal Routing/Switch', [mdl '/Live or waypoints'], ...
    'Criteria', 'u2 ~= 0', 'Position', [510 510 540 600]);
add_block('simulink/Discontinuities/Rate Limiter', [mdl '/p_ref rate limit'], ...
    'RisingSlewLimit', 'v_ref', 'FallingSlewLimit', '-v_ref', ...
    'Position', [580 558 630 592], 'BackgroundColor', 'green');
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace p_ref'], ...
    'VariableName', 'p_ref', 'SaveFormat', 'Timeseries', 'Position', [660 600 740 630]);

%% ---------------------------------------------------------------- controller
ctl = [mdl '/Controller'];
add_block('simulink/Ports & Subsystems/Subsystem', ctl, ...
    'Position', [700 330 830 430], 'BackgroundColor', 'lightBlue');
delete_line(ctl, 'In1/1', 'Out1/1');
delete_block([ctl '/In1']);
delete_block([ctl '/Out1']);

add_block('simulink/Sources/In1', [ctl '/theta'], 'Position', [20 50 50 64]);
add_block('simulink/Sources/In1', [ctl '/omega'], 'Position', [20 100 50 114]);
add_block('simulink/Sources/In1', [ctl '/p'],     'Position', [20 150 50 164]);
add_block('simulink/Sources/In1', [ctl '/v'],     'Position', [20 200 50 214]);
add_block('simulink/Sources/In1', [ctl '/p_ref'], 'Position', [20 250 50 264]);
% s = [theta; omega; x; y; vx; vy]
add_block('simulink/Signal Routing/Mux', [ctl '/state'], 'Inputs', '5', ...
    'DisplayOption', 'bar', 'Position', [100 40 105 230]);

% phi = wrap(theta - theta_ref);  s = (p - p_ref).t;  sdot = v.t
add_block('simulink/User-Defined Functions/Fcn', [ctl '/Balance  w = -Kb*[phi; omega; s; sdot]'], ...
    'Expression', ['-(Kb1*atan2(sin(u(1) - theta_ref), cos(u(1) - theta_ref))' ...
                   ' + Kb2*u(2)' ...
                   ' + Kb3*((u(3) - u(7))*cos(u(1)) + (u(4) - u(8))*sin(u(1)))' ...
                   ' + Kb4*(u(5)*cos(u(1)) + u(6)*sin(u(1))))'], ...
    'Position', [180 40 400 70]);
add_block('simulink/Discontinuities/Saturation', [ctl '/w limit'], ...
    'UpperLimit', 'amax', 'LowerLimit', '-amax', 'Position', [430 38 470 72]);

% Swing-up: horizontal energy pumping (cart-pole style) + altitude hold.
% The thrust direction only has to flip between two tilts, which the
% quadcopter can follow; a direction rotating with the rod it cannot.
addMLFcn([ctl '/Swing-up (horizontal pumping)'], [180 290 400 350], { ...
    'function a_sw = f(st, m, L, g, E_up, kE, a_sw_max, tau_p, kpx, kdx, kp, kd)'
    'th = st(1); om = st(2);'
    'E   = 0.5*m*L^2*om^2 + m*g*L*(1 - cos(th));'
    '% predict tau_p ahead to cover the attitude + motor lag'
    'omp = om - tau_p*(g/L)*sin(th);'
    'thp = th + tau_p*om;'
    '% dE/dt = -m*L*omega*cos(theta)*ax  ->  pump (or brake) with ax'
    'sw  = omp*cos(thp);'
    'if abs(sw) < 0.05, sw = 0.05; end                     % kick when hanging at rest'
    'ax  = -kE*(E_up - E)*sw;'
    'ax  = min(max(ax, -a_sw_max), a_sw_max) - kpx*(st(3) - st(7)) - kdx*st(5);'
    'ay  = -kp*(st(4) - st(8)) - kd*st(6);                % hold altitude'
    'a_sw = [ax; ay];'
    }, {'m', 'L', 'g', 'E_up', 'kE', 'a_sw_max', 'tau_p', 'kpx', 'kdx', 'kp', 'kd'});

add_block('simulink/User-Defined Functions/Fcn', [ctl '/cos(phi)'], ...
    'Expression', 'cos(u(1) - theta_ref)', 'Position', [180 90 300 120]);
% Hysteresis: enter balance at |phi| < phi_catch, leave only at |phi| > phi_release
add_block('simulink/Discontinuities/Relay', [ctl '/Catch-release (hysteresis)'], ...
    'OnSwitchValue', 'cos(phi_catch)', 'OffSwitchValue', 'cos(phi_release)', ...
    'OnOutputValue', '1', 'OffOutputValue', '0', ...
    'Position', [330 88 390 122]);
add_block('simulink/Signal Routing/Switch', [ctl '/Mode switch'], ...
    'Criteria', 'u2 ~= 0', 'Position', [860 120 900 330]);
add_block('simulink/Signal Routing/Mux', [ctl '/Mux a_bal'], 'Inputs', '2', ...
    'DisplayOption', 'bar', 'Position', [840 130 845 230]);
add_block('simulink/Signal Routing/Demux', [ctl '/Demux a'], 'Outputs', '2', ...
    'DisplayOption', 'bar', 'Position', [920 180 925 260]);

% Position hold along the rod: a_r = r.(-kp*(p - p_ref) - kd*v)
add_block('simulink/User-Defined Functions/Fcn', [ctl '/Position hold along rod'], ...
    'Expression', ['sin(u(1))*(-kp*(u(3) - u(7)) - kd*u(5))' ...
                   ' - cos(u(1))*(-kp*(u(4) - u(8)) - kd*u(6))'], ...
    'Position', [180 220 400 250]);
add_block('simulink/Discontinuities/Saturation', [ctl '/a_r limit'], ...
    'UpperLimit', 'amax', 'LowerLimit', '-amax', 'Position', [430 218 470 252]);

% a = w*t + a_r*r
add_block('simulink/Signal Routing/Mux', [ctl '/Mux a'], 'Inputs', '3', ...
    'DisplayOption', 'bar', 'Position', [660 100 665 260]);
add_block('simulink/User-Defined Functions/Fcn', [ctl '/ax'], ...
    'Expression', 'u(1)*cos(u(3)) + u(2)*sin(u(3))', 'Position', [700 130 820 160]);
add_block('simulink/User-Defined Functions/Fcn', [ctl '/ay'], ...
    'Expression', 'u(1)*sin(u(3)) - u(2)*cos(u(3))', 'Position', [700 200 820 230]);
add_block('simulink/Discontinuities/Saturation', [ctl '/ax limit'], ...
    'UpperLimit', 'amax', 'LowerLimit', '-amax', 'Position', [960 168 1000 202]);
add_block('simulink/Discontinuities/Saturation', [ctl '/ay limit (thrust >= 0)'], ...
    'UpperLimit', 'amax', 'LowerLimit', '-g', 'Position', [960 238 1000 272]);
add_block('simulink/Signal Routing/Mux', [ctl '/Mux out'], 'Inputs', '2', ...
    'DisplayOption', 'bar', 'Position', [1030 160 1035 280]);
add_block('simulink/Math Operations/Gain', [ctl '/ctrl_on'], 'Gain', 'ctrl_on', ...
    'Position', [1060 203 1100 237]);
add_block('simulink/Sinks/Out1', [ctl '/a'], 'Position', [1140 213 1170 227]);
add_block('simulink/Sinks/Out1', [ctl '/mode (1 = balance)'], 'Position', [1140 373 1170 387]);

add_line(ctl, 'theta/1', 'state/1');
add_line(ctl, 'omega/1', 'state/2');
add_line(ctl, 'p/1',     'state/3');
add_line(ctl, 'v/1',     'state/4');
add_line(ctl, 'p_ref/1', 'state/5');
for blk = {'Balance  w = -Kb*[phi; omega; s; sdot]', 'Swing-up (horizontal pumping)', ...
           'cos(phi)', 'Position hold along rod'}
    add_line(ctl, 'state/1', [blk{1} '/1'], 'autorouting', 'smart');
end
% balance: a_bal = w*t + a_r*r
add_line(ctl, 'Balance  w = -Kb*[phi; omega; s; sdot]/1', 'w limit/1');
add_line(ctl, 'w limit/1',     'Mux a/1', 'autorouting', 'smart');
add_line(ctl, 'Position hold along rod/1', 'a_r limit/1');
add_line(ctl, 'a_r limit/1',   'Mux a/2', 'autorouting', 'smart');
add_line(ctl, 'theta/1',       'Mux a/3', 'autorouting', 'smart');
add_line(ctl, 'Mux a/1', 'ax/1', 'autorouting', 'smart');
add_line(ctl, 'Mux a/1', 'ay/1', 'autorouting', 'smart');
add_line(ctl, 'ax/1', 'Mux a_bal/1', 'autorouting', 'smart');
add_line(ctl, 'ay/1', 'Mux a_bal/2', 'autorouting', 'smart');
% mode switch between the two acceleration vectors, then limits
add_line(ctl, 'Mux a_bal/1',         'Mode switch/1', 'autorouting', 'smart');
add_line(ctl, 'cos(phi)/1', 'Catch-release (hysteresis)/1');
add_line(ctl, 'Catch-release (hysteresis)/1', 'Mode switch/2', 'autorouting', 'smart');
add_line(ctl, 'Swing-up (horizontal pumping)/1', 'Mode switch/3', 'autorouting', 'smart');
add_line(ctl, 'Mode switch/1', 'Demux a/1');
add_line(ctl, 'Demux a/1', 'ax limit/1', 'autorouting', 'smart');
add_line(ctl, 'Demux a/2', 'ay limit (thrust >= 0)/1', 'autorouting', 'smart');
add_line(ctl, 'ax limit/1', 'Mux out/1', 'autorouting', 'smart');
add_line(ctl, 'ay limit (thrust >= 0)/1', 'Mux out/2', 'autorouting', 'smart');
add_line(ctl, 'Mux out/1', 'ctrl_on/1');
add_line(ctl, 'ctrl_on/1', 'a/1');
add_line(ctl, 'Catch-release (hysteresis)/1', 'mode (1 = balance)/1', 'autorouting', 'smart');

%% ---------------------------------------------------------------- sinks
add_block('simulink/Sinks/Scope', [mdl '/State scope'], 'Position', [780 40 820 80]);
% Frame sampler for live_pendulum.m: the state at fixed 20 ms hits, so a listener
% on it fires 50x per simulated second regardless of the solver's (minor) steps
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/Frame sampler'], ...
    'SampleTime', '0.02', 'Position', [880 40 910 70]);
add_block('simulink/Sinks/Terminator', [mdl '/Frame end'], 'Position', [940 45 960 65]);
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace'], ...
    'VariableName', 'x', 'SaveFormat', 'Timeseries', 'Position', [780 100 860 130]);
add_block('simulink/Sinks/XY Graph', [mdl '/Drone path'], ...
    'xmin', '-3', 'xmax', '3', 'ymin', '-3', 'ymax', '3', 'Position', [780 250 820 290]);
add_block('simulink/Signal Routing/Demux', [mdl '/Demux p'], 'Outputs', '2', ...
    'DisplayOption', 'bar', 'Position', [720 255 725 295]);
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace T'], ...
    'VariableName', 'T', 'SaveFormat', 'Timeseries', 'Position', [1600 560 1680 590]);
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace beta_des'], ...
    'VariableName', 'beta_des', 'SaveFormat', 'Timeseries', 'Position', [1370 600 1450 630]);
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace a'], ...
    'VariableName', 'a', 'SaveFormat', 'Timeseries', 'Position', [900 340 980 370]);
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace mode'], ...
    'VariableName', 'mode', 'SaveFormat', 'Timeseries', 'Position', [900 400 980 430]);
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace d'], ...
    'VariableName', 'd', 'SaveFormat', 'Timeseries', 'Position', [240 500 320 530]);

%% ---------------------------------------------------------------- lines
L_ = @(a, b) add_line(mdl, a, b, 'autorouting', 'smart');
% plant
L_('Drone + pendulum dynamics/1', 'Integrator omega/1');
L_('Drone + pendulum dynamics/2', 'Integrator v/1');
L_('Drone + pendulum dynamics/3', 'Integrator betadot/1');
L_('Integrator omega/1',   'Integrator theta/1');
L_('Integrator v/1',       'Integrator p/1');
L_('Integrator betadot/1', 'Integrator beta/1');
L_('Mux state/1',          'Drone + pendulum dynamics/1');
L_('Motor thrust/1',       'Drone + pendulum dynamics/2');
L_('pulses + noise/1',     'Drone + pendulum dynamics/3');
L_('Disturbance/1',        'pulses + noise/1');
L_('Noise/1',              'pulses + noise/2');
L_('Live kick/1',          'pulses + noise/3');
L_('pulses + noise/1',     'To Workspace d/1');
% inner loop: a_des -> F -> (T1, T2) -> motors
L_('Controller/1',         'Thrust vector (inverse dynamics)/1');
L_('Mux state/1',          'Thrust vector (inverse dynamics)/2');
L_('Thrust vector (inverse dynamics)/1', 'Attitude control + mixer/1');
L_('Mux state/1',          'Attitude control + mixer/2');
L_('Attitude control + mixer/1', 'Motor error/1');
L_('Motor thrust/1',       'Motor error/2');
L_('Motor error/1',        '1//tau_m/1');
L_('1//tau_m/1',           'Motor thrust/1');
L_('Motor thrust/1',       'To Workspace T/1');
L_('Attitude control + mixer/2', 'To Workspace beta_des/1');
L_('Integrator theta/1',   'Controller/1');
L_('Integrator omega/1',   'Controller/2');
L_('Integrator p/1',       'Controller/3');
L_('Integrator v/1',       'Controller/4');
L_('Live setpoint/1',      'Live or waypoints/1');
L_('live_on/1',            'Live or waypoints/2');
L_('Waypoints/1',          'Live or waypoints/3');
L_('Live or waypoints/1',  'p_ref rate limit/1');
L_('p_ref rate limit/1',   'Controller/5');
L_('p_ref rate limit/1',   'To Workspace p_ref/1');
L_('Controller/1',         'To Workspace a/1');
L_('Controller/2',         'To Workspace mode/1');
L_('Integrator theta/1',   'Mux state/1');
L_('Integrator omega/1',   'Mux state/2');
L_('Integrator p/1',       'Mux state/3');
L_('Integrator v/1',       'Mux state/4');
L_('Integrator beta/1',    'Mux state/5');
L_('Integrator betadot/1', 'Mux state/6');
L_('Mux state/1',          'State scope/1');
L_('Mux state/1',          'To Workspace/1');
L_('Mux state/1',          'Frame sampler/1');
L_('Frame sampler/1',      'Frame end/1');
L_('Integrator p/1',       'Demux p/1');
L_('Demux p/1',            'Drone path/1');
L_('Demux p/2',            'Drone path/2');

%% ---------------------------------------------------------------- names
nm = @(blk, k, s) set_param(getfield(get_param([mdl '/' blk], 'PortHandles'), 'Outport', {k}), 'Name', s); %#ok<GFLD>
nm('Integrator theta', 1, 'theta');
nm('Integrator omega', 1, 'omega');
nm('Integrator p',     1, 'p = [x; y]');
nm('Integrator v',     1, 'v = [vx; vy]');
nm('Integrator beta',  1, 'beta (tilt)');
nm('Integrator betadot', 1, 'betadot');
nm('Controller',       1, 'a_des = [ax; ay]');
nm('pulses + noise',   1, 'd (torque)');
nm('Mux state',        1, 'x = [theta; omega; p; v; beta; betadot]');
nm('Thrust vector (inverse dynamics)', 1, 'F = [Fx; Fy]');
nm('Attitude control + mixer', 1, 'T_cmd = [T1; T2]');
nm('Motor thrust',     1, 'T = [T1; T2]');

sc = get_param([mdl '/State scope'], 'ScopeConfiguration');
sc.ShowLegend = true;
sc.YLimits = [-8 8];
sc.TimeSpan = '25';

note = Simulink.Annotation(mdl, 'eq');
note.Text = ['Planar quadcopter (rotor pairs T1 left, T2 right) carrying an inverted pendulum.' newline ...
             'Controller -> a_des -> inverse dynamics -> attitude PD + mixer -> motors -> coupled plant.' newline ...
             'Run run_pendulum.m for plots and animation'];
note.Position = [200 640 900 690];

save_system(mdl, fullfile(here, [mdl '.slx']));
fprintf('Model saved to %s\n', fullfile(here, [mdl '.slx']));

%% ---------------------------------------------------------------- helpers
function addMLFcn(path, pos, codeLines, params)
% Adds a MATLAB Function block with the given code; names in params become
% block parameters resolved from the workspace (pendulum_params). Port sizes
% are fixed explicitly, because the feedback loops leave them underspecified.
sizes = struct('xs', '[8 1]', 'T', '[2 1]', 'a', '[2 1]', 'Fv', '[2 1]', ...
               'Tc', '[2 1]', 'pdd', '[2 1]', 'd', '1', 'thdd', '1', 'bdd', '1', ...
               'beta_des', '1', 'st', '[8 1]', 'a_sw', '[2 1]');
add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', path);
ch.Script = strjoin(codeLines, newline);
for k = 1:numel(params)
    dat = ch.find('-isa', 'Stateflow.Data', 'Name', params{k});
    dat.Scope = 'Parameter';
end
for dat = ch.find('-isa', 'Stateflow.Data')'
    if isfield(sizes, dat.Name) && ~strcmp(dat.Scope, 'Parameter')
        dat.Props.Array.Size = sizes.(dat.Name);
    end
end
end
