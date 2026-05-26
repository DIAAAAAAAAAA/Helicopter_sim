function [dx,y] = state_space_function(t,x,u,Iyy,K_pitch,m_heli, C_pitch, a, b, c ,d ,varargin)
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

% extract states
theta = x(1);
theta_dot = x(2);
I = x(3);
w = x(4);
%phi = x(3);
%phi_dot = x(4);

% Equation of motion
% pitch DOF
d_theta_dot = (K_pitch*u/Iyy) - (m_heli/Iyy)*g*L_heli*cos(theta) - (m_cw/Iyy)*g*L_cw *sin(theta) - (C_pitch / Iyy) * theta_dot;

d_I = a * I + b * u;

d_w = c + d * w;
% yaw DOF
%d_phi_dot = (K_yaw*u(2))/(Izz * cos(theta)) + 2 * tan(theta) * theta_dot * phi_dot;

% Output states
dx = [theta_dot;
    d_theta_dot;
    I;
    d_I;
    w;
    d_w];

y = [theta];



end