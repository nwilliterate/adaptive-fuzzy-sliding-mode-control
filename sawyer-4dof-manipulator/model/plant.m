% Copyright (C) 2022 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:        Jan, 18, 2022
% 
% -------------------------------------------------
% Sawyer 4 DoF Manipulator
% -------------------------------------------------
% Equation)
%       ddq = M(q)ddq + c(q, dq)dq + g(q)
%
% Input)
%       x   : Joint Postion and Velocity
%       u   : Joint Torque
% Output)
%       dxdt  : Joint Velocity and Acceleration
%
% the following code has been tested on Matlab 2021a
function dxdt = plant(q, u1,M,C,G)  

ddot = inv(M)*(-C*[q(5);q(6);q(7);q(8)]-G'+u1);
dxdt = [q(5);q(6);q(7);q(8);ddot(1);ddot(2);ddot(3);ddot(4)];

end