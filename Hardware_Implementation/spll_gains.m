function PL = spll_gains(fn, Ts, tset, zeta)
%SPLL_GAINS  Parameters of spll3ph_srf for a target settling time.
%   PL = spll_gains(446, 1e-4, 0.02, 0.7)
%   Linearized loop (normalized input, PI output in Hz):
%     s^2 + 2*pi*Kp*s + 2*pi*Ki = s^2 + 2*zeta*wn*s + wn^2,  wn = 4.6/(zeta*tset)

if nargin < 3, tset = 0.02; end
if nargin < 4, zeta = 0.7;  end
wn = 4.6/(zeta*tset);

PL.fn     = fn;
PL.Ts     = Ts;
PL.Kp     = 2*zeta*wn/(2*pi);
PL.Ki     = wn^2/(2*pi);
PL.ylfmax = 200;      % [Hz], as in TI SPLL_3PH_SRF
PL.minmag = 0.05;
end
