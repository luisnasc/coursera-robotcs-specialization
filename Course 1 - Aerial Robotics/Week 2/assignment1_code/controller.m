function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters


e = s_des(1) - s(1);
e_dot = s_des(2) - s(2);
z_des = 1;
kp=100;
kv=15;
u = 0.1800*(0.1 + kp*e + kv*e_dot + 9.81);


% FILL IN YOUR CODE HERE


end

