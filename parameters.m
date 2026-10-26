%% parameters.m  - Mini FADEC: all model and controller parameters

%% Test case (scripts may set these before calling; defaults = idle -> take-off)
if ~exist('N0',  'var'), N0  = 60; end      % initial shaft speed [%]
if ~exist('dNd', 'var'), dNd = 40; end      % demand step size [%]

%% Engine (nominal)
k = 50;                       % spool gain [%/s per unit fuel]
n = 2.7;                      % steady-state fuel map exponent
k_true = k;                   % engine actually simulated (robustness: 0.7*k, 1.3*k)

%% Operating point (derived from N0)
Wf0 = (N0/100)^n;             % equilibrium fuel at N0 [-]
a   = (n/100)*(N0/100)^(n-1); % slope of steady-state fuel curve at N0
K   = 1/a;                    % steady-state gain [% per unit fuel]
tau = 1/(k*a);                % engine time constant at N0 [s]

%% Speed controller (gain-scheduled PI)
tau_cl = 0.5;                 % design closed-loop time constant [s]
c      = 1.5;                 % K_i factor (c = 1 would be exact pole cancellation)
Kp     = 1/(k*tau_cl);        % proportional gain (same at every operating point)
Kt     = 10;                 % tracking (anti-windup) gain, scaled with c
Ts     = 0.01;                % controller sample time [s]
% Scheduled integral gain used in the model (Fcn block, u = measured N):
%   Ki(N) = c*Kp*k*(n/100)*(u/100)^(n-1)

%% Fuel schedules (protection)
m_acc = 0.20;                 % acceleration margin (surge protection)
m_dec = 0.10;                 % deceleration margin (flameout protection)
% Wf_max = (u/100)^n + m_acc,  Wf_min = (u/100)^n - m_dec

%% Fuel valve
Wf_lo     = 0.1;              % minimum fuel (flameout floor)
Wf_hi     = 1.1;              % maximum fuel
rate_lim  = 1;                % valve rate limit [1/s]
tau_valve = 0.08;             % valve lag [s] (robustness: 0.2)

%% Speed sensor
tau_sensor  = 0.02;           % sensor lag [s]
noise_power = 0;              % sensor noise power (robustness: 1e-4)

%% Open-loop tests only
dWf = 0.748;                  % fuel step for open-loop spool tests [-]