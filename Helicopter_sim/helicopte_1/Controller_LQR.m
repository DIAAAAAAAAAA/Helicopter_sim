g = 9.81;
m_cw   = 124e-3;
L_cw   = 0.02;
L_heli = 0.035;
L_m    = 0.3;

Iyy     = nl_sys_estimated.Parameters(1).Value;
m_heli  = nl_sys_estimated.Parameters(2).Value;
C_pitch = nl_sys_estimated.Parameters(3).Value;
T_bias  = nl_sys_estimated.Parameters(4).Value;
c       = nl_sys_estimated.Parameters(5).Value;
Km      = nl_sys_estimated.Parameters(6).Value;
J       = nl_sys_estimated.Parameters(7).Value;
L       = nl_sys_estimated.Parameters(8).Value;
R       = nl_sys_estimated.Parameters(9).Value;
C_T     = nl_sys_estimated.Parameters(10).Value;


theta_eq = 0;   % hover around zero pitch


Kg = g*(m_heli*L_heli + m_cw*L_cw);

w_eq_sq = (Kg*sin(theta_eq) - T_bias)/(C_T*L_m);



w_eq = sqrt(w_eq_sq);
I_eq = c*w_eq^2 / Km;
u_eq = R*I_eq + Km*w_eq;

x_eq = [theta_eq; 0; I_eq; w_eq];



A = [ 0, 1, 0, 0;
     -(Kg/Iyy)*cos(theta_eq), -(C_pitch/Iyy), 0, (2*C_T*L_m*w_eq)/Iyy;
      0, 0, -R/L, -Km/L;
      0, 0, Km/J, -(2*c*w_eq)/J ];

B = [0;
     0;
     1/L;
     0];

C = [1 0 0 0];
D = 0;

sysc = ss(A,B,C,D);

Co = ctrb(A,B);
Ob = obsv(A,C);

rank_Co = rank(Co)
rank_Ob = rank(Ob)



Q = diag([100, 10, 1, 1]);   % tune these
R_lqr = 0.1;                 % tune this

K = lqr(A,B,Q,R_lqr);




A_aug = [A, zeros(4,1);
        -C, 0];

B_aug = [B;
         0];

Q_aug = diag([100, 10, 1, 1, 200]);   % last term penalizes integral error
R_aug = 0.1;

K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);

Kx = K_aug(1:4);
Ki = K_aug(5);


u = u_eq - Kx*(x_hat - x_eq) - Ki*xi;
xi_dot = (theta_ref - theta);




cl_poles = eig(A - B*K);
obs_poles = 5*real(cl_poles);   % crude starting idea
obs_poles = [-8 -9 -10 -11];
Lobs = place(A', C', obs_poles)';
xhat_dot = A*xhat + B*u_tilde + Lobs*(y_tilde - C*xhat);