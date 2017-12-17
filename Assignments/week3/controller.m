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

u1 = 0;
u2 = 0;

% FILL IN YOUR CODE HERE
Kd_y = 2;
Kp_y = 12;

Kd_z = 14;
Kp_z = 400;

Kd_phi = 30;
Kp_phi = 2000;

phi_c = -1/params.gravity * (des_state.acc(1) ...
      + Kd_y * (des_state.vel(1) - state.vel(1)) ...
      + Kp_y * (des_state.pos(1) - state.pos(1)));
phi_c_dot = 0;
phi_c_ddot = 0;

u1 = params.mass * (params.gravity + des_state.acc(2) ...
   + Kd_z * (des_state.vel(2) - state.vel(2)) ...
   + Kp_z * (des_state.pos(2) - state.pos(2)));

u2 = params.Ixx * (phi_c_ddot + Kd_phi * (phi_c_dot - state.omega) + Kp_phi * (phi_c - state.rot));

end

