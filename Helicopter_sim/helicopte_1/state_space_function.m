function [dx,y] = state_space_function(t,x,u,Iyy,m_heli, C_pitch,T_bias, c, Km, J ,L , R,C_T ,varargin)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% state definition
% x(1): theta
% x(2): theta_dot
% x(3): phi
% x(4): phi_dot

% input definition
% u(1): voltage to pitch motor
% u(2): voltage to yaw motor
g = 9.81;
m_cw = 124e-3;      % mass of the load [kg]
L_cw = 0.02;    % im meters
L_heli = 0.035; 
L_m = 0.3;      % Distance between motor and rotational axis

% extract states
theta = x(1);
theta_dot = x(2);
I = x(3);
w = x(4);
%phi = x(3);
%phi_dot = x(4);
T = C_T * w^2;


% Equation of motion
% pitch DOF
d_theta_dot = (T*L_m/Iyy) - (m_heli/Iyy)*g*L_heli*sin(theta) - (m_cw/Iyy)*g*L_cw *sin(theta) - (C_pitch / Iyy) * theta_dot...
    + (T_bias / Iyy);



d_I = -R / L * I - Km / L * w  + u / L ;

d_w = Km * I / J - c * w^2 / J;
% yaw DOF
%d_phi_dot = (K_yaw*u(2))/(Izz * cos(theta)) + 2 * tan(theta) * theta_dot * phi_dot;

% Output states
dx = [theta_dot;
    d_theta_dot;
    d_I;
    d_w];

y = [theta];



end