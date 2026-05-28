function [dx,y] = state_space_function(t,x,u,Iyy,K_pitch,m_heli,L_heli,L_cw,C_pitch,K_yaw, Izz, C_yaw,varargin)
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

% extract states
theta = x(1);
theta_dot = x(2);
phi = x(3);
phi_dot = x(4);

% Equation of motion
% pitch DOF
d_theta_dot = (K_pitch*u(1)/Iyy)- Izz * cos(theta) *sin(theta)* phi_dot^2 / Iyy - (m_heli/Iyy)*g*L_heli*cos(theta) - (m_cw/Iyy)*g*L_cw *sin(theta)- (C_pitch/Iyy)*theta_dot;

% yaw DOF
d_phi_dot = (K_yaw*u(2))/(Izz * cos(theta)) + 2 * tan(theta) * theta_dot * phi_dot -  (C_yaw/Izz)*phi_dot;

% Output states
dx = [theta_dot;
    d_theta_dot;
    phi_dot;
    d_phi_dot];

y = [theta, phi];

%{
% Limit theta to [-60, 60] degrees (convert to radians)
theta_max = deg2rad(60);
theta_min = deg2rad(-60);

if theta >= theta_max && d_theta_dot > 0
    theta_dot = 0;
    d_theta_dot = 0;
elseif theta <= theta_min && d_theta_dot < 0
    theta_dot = 0;
    d_theta_dot = 0;
end
%}

end