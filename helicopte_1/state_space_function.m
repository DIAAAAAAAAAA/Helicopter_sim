function [dx,y] = state_space_function(t,x,u, Iyy, m_heli, C_pitch, R, L, Kt, Ke, Jm, bm, kq, kt, varargin)

g = 9.81;
L_heli = 0.035 ;
m_cw = 124e-3;
L_cw = 0.02 ;

% States
theta     = x(1);
theta_dot = x(2);
I         = x(3);
omega     = x(4);

% Input
u = u(1);

% Motor electrical
d_I = (u - R*I - Ke*omega) / L;

% Prop torque load (quadratic)
tau_prop = kq * omega^2;

% Motor mechanical
d_omega = (Kt*I - bm*omega - tau_prop) / Jm;
d

% Thrust (quadratic)
T = kt * omega^2;

% Pitch acceleration
d_theta_dot = (L_heli*T  - m_heli*g*L_heli*cos(theta) - m_cw*g*L_cw*sin(theta) - C_pitch*theta_dot) / Iyy;

% State derivatives
dx = [theta_dot;
      d_theta_dot;
      d_I;
      d_omega];

% Output
y = theta;



dx = dx(:);

y  = y(:);

end