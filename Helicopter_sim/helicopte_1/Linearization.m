g = 9.81;
m_cw   = 124e-3;
L_cw   = 0.02;
L_heli = 0.035;
L_m    = 0.3;

Iyy     = 0.039857;
m_heli  = 0.795216;
C_pitch = 0.0039208;
T_bias  = -0.00916309;
c       = 0.000450553;
Km      = 0.00232502;
J       = 0.000131343;
L       = 0.0148128;
R       = 4.12861;
C_T     = -0.266161;   % use with care


theta0 = 0;    % rad
theta_dot0 = 0;


Kg = g*(m_heli*L_heli + m_cw*L_cw);

w0_sq = (Kg*sin(theta0) - T_bias)/(C_T*L_m);


w0 = sqrt(w0_sq);
I0 = c*w0^2 / Km;
u0 = R*I0 + Km*w0;

x0 = [theta0; theta_dot0; I0; w0];


[A,B,C,D] = linmod('helicoptertemplate', x0, u0);

