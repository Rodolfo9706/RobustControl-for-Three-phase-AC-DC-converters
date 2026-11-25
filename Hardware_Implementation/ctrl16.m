function [p, dghat, dnu, gr, e, s] = ctrl16(x, ghat, nu, vod, w, P)
%CTRL16  Adaptive passivity-based controller, law (16) of TCST 26-1303.
%
%   Inputs
%     x     = [id; iq; vo]  measured dq currents [A] and dc-link voltage [V]
%     ghat  = conductance estimate [S]      (state, integrate dghat outside)
%     nu    = filtered voltage error [V]    (state, integrate dnu outside)
%     vod   = dc-link voltage reference [V]
%     w     = source angular frequency [rad/s] (= 2*pi*Freq from the PLL)
%     P     = parameter struct (see params_tcst.m), MATLAB Function
%             "Parameter" data, non-tunable
%
%   Outputs
%     p     = [pd; pq]  saturated modulation indices, p in S = [-ad,ad]x[-aq,aq]
%     dghat = time derivative of ghat, projected update (16d)
%     dnu   = time derivative of nu, first-order filter (16c)
%     gr    = reference conductance (16c) [S]
%     e     = [e1; e2; e3] errors (6)
%     s     = passive-output component s = I(gr) e3 - vod e1, see (9)
%
%   Notation and equation numbers follow the revised manuscript.

id = x(1);  iq = x(2);  vo = x(3);

% ---- Reference conductance (16c) ---------------------------------------
gr = ghat - P.kappa*P.nubar*tanh(nu/P.nubar);

% ---- Equilibrium current and feedforward, Eqs. (2) and (4) -------------
Ir   = eqCurrent(gr, vod, P);
pdeq =  2*(P.Em - P.R*Ir)/vod;
pqeq = -2*P.L*w*Ir/vod;

% ---- Errors (6) and passive output (9) ---------------------------------
e1 = id - Ir;
e2 = iq;
e3 = vo - vod;
s  = Ir*e3 - vod*e1;

% ---- Channel-wise saturated port law (16a)-(16b) -----------------------
pd = sat(pdeq - (2*P.ks/3)*s,        P.ad);
pq = sat(pqeq + (2*P.kq/(3*vod))*iq, P.aq);
p  = [pd; pq];

% ---- Filter (16c) and projected estimator (16d) ------------------------
dnu   = P.lambdaf*(e3 - nu);
y     = -2*P.gamma*vod*e3 - P.gamma*P.sigma*(ghat - P.gc);
dghat = proj(ghat, y, P.glow, P.gup, P.epsp);

e = [e1; e2; e3];
end

% =========================================================================
function I = eqCurrent(g, vod, P)
% Smaller root of the power balance (3), Eq. (4). The argument of sqrt is
% positive whenever g < gmax, which (17) guarantees on Gr; it is clipped
% only to protect the simulation if (17) is violated.
arg = P.Em^2 - (8/3)*P.R*vod^2*g;
I   = (P.Em - sqrt(max(arg, 0)))/(2*P.R);
end

function v = sat(u, a)
% Saturation onto [-a, a].
v = min(max(u, -a), a);
end

function py = proj(g, y, glow, gup, ep)
% Smooth projection (14): keeps g in [glow-ep, gup+ep].
cp = min(1, max(0, (g - gup)/ep));
cm = min(1, max(0, (glow - g)/ep));
py = y - cp*max(y, 0) - cm*min(y, 0);
end
