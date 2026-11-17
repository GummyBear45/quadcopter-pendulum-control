%% Parameters of the rigid pendulum (point mass on a massless rigid rod)
% Called automatically by the Simulink model's InitFcn callback.

m = 0.3;            % mass at the end of the rod            [kg]  (lighter -> less rod pull on the drone)
L = 1.0;            % rod length (pivot -> mass)            [m]
g = 9.81;           % gravitational acceleration            [m/s^2]
b = 0.15*m;         % viscous damping at the pivot          [N*m*s/rad]  (scaled with m: same
                    % b/(m*L^2); 0.15 with 300 g makes the drone's tilting drag the rod into a limit cycle)

theta0 = 0;             % initial angle from the downward vertical [rad] (0 = hanging, 180 deg = upright)
omega0 = 0;             % initial angular velocity               [rad/s]

Tsim = 25;          % simulation time [s]

%% Controller
%   balance  (entered at |phi| < phi_catch, left at |phi| > phi_release):
%       a = w*t + a_r*r,  w = -Kb*[phi; omega; s; sdot]   (see Drone)
%   swing-up (otherwise): horizontal pumping + altitude hold
%       ax = sat(-kE*(E_up - E)*omega_p*cos(theta_p), a_sw_max) - kpx*(x - x_ref(t)) - kdx*vx
%       ay = -kp*(y - y_ref(t)) - kd*vy
ctrl_on   = 1;      % 1 = controller active, 0 = free pendulum
theta_ref = pi;     % upright equilibrium [rad]

% Swing-up (energy pumping with horizontal drone acceleration, like a cart-pole)
E_up      = 2*m*g*L;        % energy of the upright position at rest [J]
kE        = 2/m;            % energy-pumping gain [m/s^2 per J*rad/s] (E scales with m)
a_sw_max  = 5;              % max horizontal pumping acceleration [m/s^2] (~27 deg tilt)
kpx = 1;  kdx = 1.5;        % weak horizontal centring during swing-up
phi_catch   = deg2rad(50);  % switch to balance when |theta - pi| < phi_catch
phi_release = deg2rad(80);  % ...and back to swing-up only when |theta - pi| > phi_release
                            % (hysteresis: no chattering at the edge of the window)
tau_p     = 0;              % prediction horizon [s] for the pump direction (omega and
                            % theta predicted tau_p ahead); 0 works best with tau_ff below

% Desired closed-loop behaviour of the linearised system around the upright position
wn   = 4;           % natural frequency [rad/s]
zeta = 0.8;         % damping ratio     [-]

%% Drone (the pivot is carried by a drone that can accelerate in x and y)
% Pendulum:  theta'' = -[(g + ay)*sin(theta) + ax*cos(theta)]/L - b/(m*L^2)*omega + d/(m*L^2)
% Only the acceleration component along t = (cos(theta), sin(theta)) (perpendicular
% to the rod) rotates the pendulum: w = a.t.  The component along the rod
% r = (sin(theta), -cos(theta)) is used to hold the drone's position.
x0 = 0;  y0 = 0;            % initial drone position [m]
% Scripted waypoints for the drone position: [t x y], the reference jumps to
% (x, y) at time t and is held; a rate limiter turns the jumps into ramps.
% The moves start after the catch (~3 s) so swing-up happens around (0, 0).
waypoints = [ 0   0   0
              8   1   0
             14   1   1
             20   0   0];
v_ref   = 0.5;              % max reference speed per axis [m/s]
wp_data = waypoints;        % table for the From Workspace block
live_on = 0;                % 1 = setpoint from p_live (arrow keys, live_pendulum.m)
p_live  = [0; 0];           % live setpoint [x; y] [m]; tunable while the model runs
kick_live = 0;              % live push torque [N*m] (set by the buttons in live_pendulum.m)
amax  = 15;                 % max acceleration per axis [m/s^2]; ay is also limited to >= -g
kp = 4;  kd = 4;            % position hold along the rod (PD)

%% Quadcopter body (planar: 4 rotors in 2 pairs, left pair T1 at -l_arm, right pair T2 at +l_arm)
% The outer controller still asks for a drone acceleration a_des. Below it:
%   inverse dynamics -> thrust vector F -> desired tilt beta_des -> attitude PD
%   -> mixer (T1, T2) -> motors (first-order lag, 0..Tmax) -> coupled rigid-body plant
M      = 1.5;       % drone mass                         [kg]
J      = 0.03;      % drone moment of inertia            [kg*m^2]
l_arm  = 0.25;      % distance from centre to each rotor pair [m]
Tmax   = 25;        % max thrust of one rotor pair       [N]  (thrust/weight = 2*Tmax/((M+m)*g) ~ 2)
tau_m  = 0.02;      % motor (thrust) time constant       [s]
beta0  = 0;         % initial tilt                       [rad]
beta_max = deg2rad(60);   % tilt command limit           [rad]
T_hover  = (M + m)*g/2;   % thrust per pair in hover     [N]

% Attitude PD: J*beta'' = tau,  tau = -kb_att*(beta - beta_des) - kbd_att*beta'
wn_att   = 25;  zeta_att = 0.8;   % ~6x faster than the balance loop (wn = 4);
                                  % at 15 rad/s the drone can no longer balance
kb_att   = J*wn_att^2;
kbd_att  = 2*zeta_att*wn_att*J;
tau_ff   = 0.05;    % feed-forward prediction [s]: compute the rod pull this far ahead.
                    % 0 -> the late cancellation damps the swing; > 0.06 -> it pumps
                    % energy in and goes unstable. 0.05 is ~energy-neutral here.

% Balance: full state feedback w = -Kb*[phi; omega; s; sdot]
%   s = (p - p_ref).t  (drone offset perpendicular to the rod)
% Linearised around upright:  phi'' = (g/L)*phi - w/L,   s'' = w
A  = [0 1 0 0; g/L 0 0 0; 0 0 0 1; 0 0 0 0];
B  = [0; -1/L; 0; 1];
wn_p = 1.2;  zeta_p = 0.9;  % slower pole pair for the drone position
pc = conv([1 2*zeta*wn wn^2], [1 2*zeta_p*wn_p wn_p^2]);    % desired char. polynomial
Kb = [0 0 0 1] / [B A*B A^2*B A^3*B] * polyvalm(pc, A);      % Ackermann's formula
Kb1 = Kb(1); Kb2 = Kb(2); Kb3 = Kb(3); Kb4 = Kb(4);          % scalars for the Fcn block

%% Disturbance: rectangular torque pulses acting on the pendulum (added to u)
dist_on  = 1;               % 1 = apply disturbance, 0 = none
dist_t   = [7    13];       % pulse start times [s]
dist_amp = [0     0];       % pulse amplitudes  [N*m]  (+ pushes theta up); try [3 -15]
dist_dur = [0.2  0.3];      % pulse durations   [s]
%   pulse 1: small push while balancing  -> balance controller absorbs it
%   pulse 2: big hit                     -> knocked out, swing-up recovers

% Piecewise-constant table [time, torque] for the From Workspace block
dist_data = [0 0];
for i = 1:numel(dist_t)
    dist_data = [dist_data;                              %#ok<AGROW>
                 dist_t(i)               dist_on*dist_amp(i);
                 dist_t(i) + dist_dur(i) 0];
end
dist_data = [dist_data; Tsim 0];

% Random noise torque added on top of the pulses (Gaussian, zero-order hold)
noise_std  = 0.0*m;         % standard deviation [N*m]   (0 = no noise); scaled with m -> same angular kick
noise_Ts   = 0.05;          % sample time: a new random value every noise_Ts [s]
noise_seed = 42;            % fixed seed -> the same noise each run; change for a new realisation
noise_std  = dist_on * noise_std;
