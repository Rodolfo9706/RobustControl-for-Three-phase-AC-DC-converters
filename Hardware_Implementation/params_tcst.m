%PARAMS_TCST  Experimental parameters for the hardware implementation of the
%   controller (16), TCST 26-1303 (Section V, Tables IV-V).
%   Run before building/deploying the LAUNCHXL-F28379D model. In the MATLAB
%   Function blocks declare P (ctrl16) and PL (spll3ph_srf) as non-tunable
%   parameters.

GAINS = 'tableV';      % 'tableV' | 'certified'

% ---- Converter, Table IV -------------------------------------------------
P.Em = 12;             % fundamental peak phase voltage [V]   (CHECK: measured value)
P.R  = 0.1;            % line resistance [Ohm]
P.L  = 2e-3;           % line inductance [H]
P.C  = 2.5e-3;         % dc-link capacitance [F]
fsrc = 446;            % source frequency [Hz] (BLDC generator, approximate)
Ts   = 1e-4;           % sampling period [s] (10 kHz)

% ---- Saturation set S, Eq. (5) -------------------------------------------
P.ad = 1.013;
P.aq = 0.55;

% ---- Projection (14) and sigma-modification, Table V ---------------------
P.glow  = 0.01;        % [S]
P.gup   = 0.025;       % [S]
P.epsp  = 1e-3;        % [S]
P.gc    = 1/45;        % sigma-modification center [S]
P.sigma = 0.01;

% ---- Port damping, Table V -----------------------------------------------
P.ks = 0.1;
P.kq = 100;

% ---- Estimator and filtered proportional action --------------------------
switch GAINS
    case 'tableV'      % Table V: performance tuning, violates (18), and (17) at 29 V
        P.gamma = 2.5e-3;  P.kappa = 8e-3;  P.lambdaf = 1500;  P.nubar = 1;
    case 'certified'   % satisfies (17) and (18) for vod in [24, 29] V;
                       % sigma = 0.01 < sigma* (~7e5 at 29 V), Theorem 1(ii)
        P.gamma = 6e-4;    P.kappa = 2e-3;  P.lambdaf = 200;   P.nubar = 0.5;
end

% ---- PLL (spll3ph_srf): settling time 20 ms, damping 0.7 -----------------
PL = spll_gains(fsrc, Ts, 0.02, 0.7);

% ---- Initial conditions of the controller states -------------------------
ghat0 = P.gc;          % must lie in G_eps = [glow - epsp, gup + epsp]
nu0   = 0;
