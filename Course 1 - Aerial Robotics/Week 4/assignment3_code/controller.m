function [F, M] = controller(t, state, stateDes, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thrust
F = 0;

% Moment
M = zeros(3,1);






m = params.mass;
g = params.gravity;

% Gains for the thrust
kpLinear = 20; % Try to fill this in
kdLinear = 5; % Try to fill this in
% Gains for the moment
kpRotation = 70; % Try to fill this in
kdRotation = 5; % Try to fill this in

omegaDes = [0; 0; stateDes.yawdot];



tHat = stateDes.vel/norm(stateDes.vel);
nHat = stateDes.acc/norm(stateDes.acc);
bHat = cross(tHat, nHat);
deltaP = stateDes.pos - state.pos;

ep = dot(deltaP, nHat)*nHat + dot(deltaP, bHat)*bHat;

% handling for nan values in bHat
if (any(isnan(bHat)))  % Handling for nan in bHat
    ep = stateDes.pos - state.pos;
end

ev =  stateDes.vel - state.vel;

ddotRdes = stateDes.acc + kdLinear * ev + kpLinear * ep;

F = m*g + m*ddotRdes(3);


psiDes = stateDes.yaw;
phiDes = (1/g)*(ddotRdes(1)*sin(psiDes) - ddotRdes(2)*cos(psiDes));


thetaDes = (1/g)*(ddotRdes(1)*cos(psiDes) + ddotRdes(2)*sin(psiDes));

rotDes = [phiDes; thetaDes; psiDes];


M = kpRotation * (rotDes - state.rot) + kdRotation * (omegaDes - state.omega);
% =================== Your code ends here ===================

end
