function [wt, freq] = spll3ph_srf(abc, PL)
%SPLL3PH_SRF  Three-phase synchronous-reference-frame PLL, discrete.
%   Same structure as TI C2000Ware SPLL_3PH_SRF (spll_3ph_srf.h):
%     v_q -> PI loop filter (Tustin, coefficients b0, b1) -> fo = fn + ylf [Hz]
%     -> theta(k) = theta(k-1) + 2*pi*fo*Ts, wrapped to [0, 2*pi).
%   The input vector is normalized, so the loop gain does not depend on its
%   amplitude (TI expects per-unit voltages).
%
%   Inputs
%     abc = [a; b; c]  synchronizing three-phase signal (voltages or currents)
%     PL  = parameter struct (MATLAB Function "Parameter", non-tunable):
%           PL.fn      nominal frequency [Hz]          (e.g. 446)
%           PL.Ts      sample time [s]                 (1e-4)
%           PL.Kp, PL.Ki  PI gains, output in Hz       (see spll_gains below)
%           PL.ylfmax  max frequency deviation [Hz]    (TI uses 200)
%           PL.minmag  min |x_alphabeta| to update     (e.g. 0.05)
%
%   Outputs
%     wt   = estimated angle theta in [0, 2*pi) [rad], d axis aligned with abc
%     freq = estimated frequency fo [Hz]  (w = 2*pi*freq for ctrl16)
%
%   Block settings: discrete, Sample time = PL.Ts.

persistent theta ylf vq1
if isempty(theta)
    theta = 0;  ylf = 0;  vq1 = 0;
end

% Clarke, amplitude invariant (three-wire)
xa = abc(1);
xb = (abc(1) + 2*abc(2))/sqrt(3);

% Park with the current angle; q component of the normalized vector
mag = sqrt(xa^2 + xb^2);
if mag > PL.minmag
    vq = (-xa*sin(theta) + xb*cos(theta))/mag;     % = sin(theta_x - theta)
else
    vq = 0;                                       % free-running at fn
end

% Loop filter (TI form): ylf(k) = ylf(k-1) + b0*vq(k) + b1*vq(k-1)
b0 =  PL.Kp + PL.Ki*PL.Ts/2;
b1 = -PL.Kp + PL.Ki*PL.Ts/2;
ylf = ylf + b0*vq + b1*vq1;
ylf = min(max(ylf, -PL.ylfmax), PL.ylfmax);
vq1 = vq;

% VCO
freq  = PL.fn + ylf;
theta = theta + 2*pi*freq*PL.Ts;
if theta >= 2*pi, theta = theta - 2*pi; end
if theta < 0,     theta = theta + 2*pi; end

wt = theta;
end
