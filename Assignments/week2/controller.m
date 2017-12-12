function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;

% FILL IN YOUR CODE HERE
%  z'' = u/m - g
%   u = m(z''_des + K_p e + K_v e' + g)
%   e(t) = z^des(t) - z(t)
K_p = 130;
K_v = 20;

a_z = 0;
e = s_des - s;
u = params.mass * (a_z + K_v * e(2) + K_p * e(1) + params.gravity);



end