function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

m = params.mass;%0.1800;
g = params.gravity;%9.8100;

y = state.pos(1);
z = state.pos(2);

y_dot = state.vel(1);
z_dot = state.vel(2);

y_des = des_state.pos(1);
z_des = des_state.pos(2);

phi = state.rot;
phi_dot = state.omega;

%%%%

y_dot_des = des_state.vel(1);
z_dot_des = des_state.vel(2);

y_ddot = des_state.acc(1);
z_ddot = des_state.acc(2);

% FILL IN YOUR CODE HERE
kpy=20;
kdy=5;
phi_c = (-1*( y_ddot + kdy*(y_dot_des-y_dot) + kpy*(y_des - y)))/g;

kph=60;
kdh=5;
u2 = kph*(phi_c - phi) + kdh*(phi_c - phi_dot);

kpz = 100;
kdz = 5;
u1 = m*(g+z_ddot+kdz*(z_dot_des-z_dot)+kpz*(z_des-z));

end

