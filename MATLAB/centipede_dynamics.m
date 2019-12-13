function dxdt = centipede_dynamics(t,x)
tau = Controller(t,x);
q = x(1:21);
dq = x(22:42);
D1 = Massmat(q);
C1 = Corvec(x);
G1 = Gravvec(q);
B1 = Torquemat();
Jll = footjacll(q);
Jlr = footjaclr(q);
Jrl = footjacrl(q);
Jrr = footjacrr(q);
dJll = footdjacll(x);
dJlr = footdjaclr(x);
dJrl = footdjacrl(x);
dJrr = footdjacrr(x);

Massmatappended = [D1 -Jll' -Jlr' -Jrl' -Jrr';...
                 Jll zeros(3,12);...
                 Jlr zeros(3,12);...
                 Jrl zeros(3,12);...
                 Jrr zeros(3,12)];
             
mat1 = [B1*tau-C1*dq-G1; -dJll*dq;-dJlr*dq;-dJrl*dq;-dJrr*dq];
v = [eye(21) zeros(21,length(mat1)-21)]*(Massmatappended\mat1);
dxdt = [dq;v];
end