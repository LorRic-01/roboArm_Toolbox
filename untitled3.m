A_fun = arm.A_genFun;

import casadi.*

k = 3;
x = casadi.MX.get_input(A_fun{k}); x = x{1};

A = A_fun{k}(x);
T = A(1:3, 4);
T_dx = jacobian(T, x);

T_fun = Function('T_fun', {x}, {T});
T_dxfun = Function('T_dxfun', {x}, {T_dx});