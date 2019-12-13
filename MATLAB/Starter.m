clc 
clear

if ~exist('./gen')
    mkdir('./gen')
end
addpath('./gen')
%% System Parameters
% syms g m l real
g = 9.81;
m = 100; %mass of the main body - there are 2 such bodies connected
l = 0.5; %distance to the center of mass of each body from the connecting point
         %also the distance from the center of mass to the point where the
         %leg is connected
r = 0.5; %length of each leg, each leg is assumed to be a lumped mass at the center of the link
ml = 5; %mass of each link of the leg

MoIltensor = diag([10 10 20]); %moment of inertia tensor in the body frame of each main body
MoIrtensor = diag([10 10 20]);


%%
syms x y z real %position of the joint at which both the bodies are connected
syms ql1 ql2 ql3 real %euler angles of the left body in the order psi theta phi
syms qr1 qr2 qr3 real %euler angles of the right body in the order psi theta phi

syms all1 all2 all3 real %relative angles of the leg links - left body left leg
syms alr1 alr2 alr3 real %relative angles of the leg links - left body right leg
syms arl1 arl2 arl3 real %relative angles of the leg links - right body left leg
syms arr1 arr2 arr3 real %relative angles of the leg links - right body right leg

syms dx dy dz dql1 dql2 dql3 dqr1 dqr2 dqr3 real

syms dall1 dall2 dall3 real
syms dalr1 dalr2 dalr3 real
syms darl1 darl2 darl3 real           %all their derivatives
syms darr1 darr2 darr3 real

%% Generalized coords and velocities
% Generalized Coordinates
q = [x;y;z;...
    ql1;ql2;ql3;...
    qr1;qr2;qr3;...
    all1;all2;all3;...
    alr1;alr2;alr3;...
    arl1;arl2;arl3;...
    arr1;arr2;arr3] ;
% Generalized Velocities
dq = [dx;dy;dz;...
    dql1;dql2;dql3;...
    dqr1;dqr2;dqr3;...
    dall1;dall2;dall3;...
    dalr1;dalr2;dalr3;...
    darl1;darl2;darl3...
    darr1;darr2;darr3];
%% Orientation matrices of the left body 
Ol1w = [cos(ql1) sin(ql1) 0;...
      -sin(ql1) cos(ql1) 0;...   % Orientation matrix --- rotation about z axis by ql1
      0 0 1];
Ol21 = [cos(ql2) 0 -sin(ql2);...
      0 1 0;...                  % Orientation matrix --- rotation about y axis by ql2
      sin(ql2) 0 cos(ql2)];
Ol32 = [1 0 0;...
      0 cos(ql3) sin(ql3);...    % Orientation matrix --- rotation about x axis by ql3
      0 -sin(ql3) cos(ql3)];
%% Orientation matrices of the right body 
Or1w = [cos(qr1) sin(qr1) 0;...
      -sin(qr1) cos(qr1) 0;...
      0 0 1];
Or21 = [cos(qr2) 0 -sin(qr2);...
      0 1 0;...
      sin(qr2) 0 cos(qr2)];
Or32 = [1 0 0;...
      0 cos(qr3) sin(qr3);...
      0 -sin(qr3) cos(qr3)];
%% Orientation matrix of the left body with respect to the world frame attached at [x;y;z]
Olw = simplify(Ol32*Ol21*Ol1w);
%% Orientation matrix of the right body with respect to the world frame attached at [x;y;z]
Orw = simplify(Or32*Or21*Or1w);
%% Orientation matrices of the legs with respect to the world frame
%left body and left leg links
Oll1w = simplify([cos(all1) sin(all1) 0; -sin(all1) cos(all1) 0;0 0 1]*Olw);
Oll2w = simplify([1 0 0;...
      0 cos(all2) sin(all2);...
      0 -sin(all2) cos(all2)]*Oll1w);
Oll3w = simplify([1 0 0;...
      0 cos(all3) sin(all3);...
      0 -sin(all3) cos(all3)]*Oll2w);

%left body and right leg links
Olr1w = simplify([cos(alr1) sin(alr1) 0; -sin(alr1) cos(alr1) 0;0 0 1]*Olw);
Olr2w = simplify([1 0 0;...
      0 cos(alr2) sin(alr2);...
      0 -sin(alr2) cos(alr2)]*Olr1w);
Olr3w = simplify([1 0 0;...
      0 cos(alr3) sin(alr3);...
      0 -sin(alr3) cos(alr3)]*Olr2w);

%right body and left leg links
Orl1w = simplify([cos(arl1) sin(arl1) 0; -sin(arl1) cos(arl1) 0;0 0 1]*Orw);
Orl2w = simplify([1 0 0;...
      0 cos(arl2) sin(arl2);...
      0 -sin(arl2) cos(arl2)]*Orl1w);
Orl3w = simplify([1 0 0;...
      0 cos(arl3) sin(arl3);...
      0 -sin(arl3) cos(arl3)]*Orl2w);

%right body and right leg links
Orr1w = simplify([cos(arr1) sin(arr1) 0; -sin(arr1) cos(arr1) 0;0 0 1]*Orw);
Orr2w = simplify([1 0 0;...
      0 cos(arr2) sin(arr2);...
      0 -sin(arr2) cos(arr2)]*Orr1w);
Orr3w = simplify([1 0 0;...
      0 cos(arr3) sin(arr3);...
      0 -sin(arr3) cos(arr3)]*Orr2w);

%% Position vectors and velocities
plcom = [x;y;z]+Olw'*[-l;0;0]; %com position of left body

pll1  = plcom+Olw'*[-l;0;0]+Oll1w'*[0;-r/2;0]; %com position of first link of left leg of left body 
pll2  = plcom+Olw'*[-l;0;0]+Oll1w'*[0;-r;0]+Oll2w'*[0;-r/2;0];
pll3  = plcom+Olw'*[-l;0;0]+Oll1w'*[0;-r;0]+Oll2w'*[0;-r;0]+Oll3w'*[0;-r/2;0];
pllst = plcom+Olw'*[-l;0;0]+Oll1w'*[0;-r;0]+Oll2w'*[0;-r;0]+Oll3w'*[0;-r;0]; %point wher the leg touches the ground

plr1  = plcom+Olw'*[-l;0;0]+Olr1w'*[0;r/2;0];
plr2  = plcom+Olw'*[-l;0;0]+Olr1w'*[0;r;0]+Olr2w'*[0;r/2;0];
plr3  = plcom+Olw'*[-l;0;0]+Olr1w'*[0;r;0]+Olr2w'*[0;r;0]+Olr3w'*[0;r/2;0];
plrst = plcom+Olw'*[-l;0;0]+Olr1w'*[0;r;0]+Olr2w'*[0;r;0]+Olr3w'*[0;r;0];

vlcom = simplify(jacobian(plcom,q)*dq);

vll1  = simplify(jacobian(pll1,q)*dq);
vll2  = simplify(jacobian(pll2,q)*dq); %absolute velocities
vll3  = simplify(jacobian(pll3,q)*dq);

vlr1  = simplify(jacobian(plr1,q)*dq);
vlr2  = simplify(jacobian(plr2,q)*dq);
vlr3  = simplify(jacobian(plr3,q)*dq);



prcom = [x;y;z]+Orw'*[l;0;0];

prl1  = prcom+Orw'*[l;0;0]+Orl1w'*[0;-r/2;0];
prl2  = prcom+Orw'*[l;0;0]+Orl1w'*[0;-r;0]+Orl2w'*[0;-r/2;0];
prl3  = prcom+Orw'*[l;0;0]+Orl1w'*[0;-r;0]+Orl2w'*[0;-r;0]+Orl3w'*[0;-r/2;0];
prlst = prcom+Orw'*[l;0;0]+Orl1w'*[0;-r;0]+Orl2w'*[0;-r;0]+Orl3w'*[0;-r;0];

prr1  = prcom+Orw'*[l;0;0]+Orr1w'*[0;r/2;0];
prr2  = prcom+Orw'*[l;0;0]+Orr1w'*[0;r;0]+Orr2w'*[0;r/2;0];
prr3  = prcom+Orw'*[l;0;0]+Orr1w'*[0;r;0]+Orr2w'*[0;r;0]+Orr3w'*[0;r/2;0];
prrst = prcom+Orw'*[l;0;0]+Orr1w'*[0;r;0]+Orr2w'*[0;r;0]+Orr3w'*[0;r;0];

vrcom = simplify(jacobian(prcom,q)*dq);

vrl1  = simplify(jacobian(prl1,q)*dq);
vrl2  = simplify(jacobian(prl2,q)*dq);
vrl3  = simplify(jacobian(prl3,q)*dq);

vrr1  = simplify(jacobian(prr1,q)*dq);
vrr2  = simplify(jacobian(prr2,q)*dq);
vrr3  = simplify(jacobian(prr3,q)*dq);
%% Angular velocity calculation of the bodies
Olt = Olw';
dOl1t = simplify(jacobian(Olt(1,:),[ql1 ql2 ql3])*[dql1;dql2;dql3]);
dOl2t = simplify(jacobian(Olt(2,:),[ql1 ql2 ql3])*[dql1;dql2;dql3]);
dOl3t = simplify(jacobian(Olt(3,:),[ql1 ql2 ql3])*[dql1;dql2;dql3]);
Ort = Orw';
dOr1t = simplify(jacobian(Ort(1,:),[qr1 qr2 qr3])*[dqr1;dqr2;dqr3]);
dOr2t = simplify(jacobian(Ort(2,:),[qr1 qr2 qr3])*[dqr1;dqr2;dqr3]);
dOr3t = simplify(jacobian(Ort(3,:),[qr1 qr2 qr3])*[dqr1;dqr2;dqr3]);



dOl = [dOl1t'; dOl2t'; dOl3t'];
dOr = [dOr1t'; dOr2t'; dOr3t'];

Angvellmat = simplify(Olw*dOl);
Angvelrmat = simplify(Orw*dOr);

Angvell = [-Angvellmat(2,3);Angvellmat(1,3);-Angvellmat(1,2)];
Angvelr = [-Angvelrmat(2,3);Angvelrmat(1,3);-Angvelrmat(1,2)];

%% Potential and Kinetic Energies
PE1 = m*g*plcom(3);
PE2 = m*g*prcom(3);

PEll = ml*g*(pll1(3)+pll2(3)+pll3(3));
PElr = ml*g*(plr1(3)+plr2(3)+plr2(3));
PErl = ml*g*(prl1(3)+prl2(3)+prl2(3));
PErr = ml*g*(prr1(3)+prr2(3)+prr3(3));

PE = simplify(PE1+PE2+PEll+PElr+PErl+PErr);

KE1 = simplify(0.5*m*(vlcom(1)^2+vlcom(2)^2+vlcom(3)^2)+0.5*Angvell'*MoIltensor*Angvell);
KE2 = simplify(0.5*m*(vrcom(1)^2+vrcom(2)^2+vrcom(3)^2)+0.5*Angvelr'*MoIrtensor*Angvelr);

KEll = 0.5*ml*(vll1(1)^2+vll1(2)^2+vll1(3)^2)+...
       0.5*ml*(vll2(1)^2+vll2(2)^2+vll2(3)^2)+...
       0.5*ml*(vll3(1)^2+vll3(2)^2+vll3(3)^2);
KElr = 0.5*ml*(vlr1(1)^2+vlr1(2)^2+vlr1(3)^2)+...
       0.5*ml*(vlr2(1)^2+vlr2(2)^2+vlr2(3)^2)+...
       0.5*ml*(vlr3(1)^2+vlr3(2)^2+vlr3(3)^2);
KErl = 0.5*ml*(vrl1(1)^2+vrl1(2)^2+vrl1(3)^2)+...
       0.5*ml*(vrl2(1)^2+vrl2(2)^2+vrl2(3)^2)+...
       0.5*ml*(vrl3(1)^2+vrl3(2)^2+vrl3(3)^2);
KErr = 0.5*ml*(vrr1(1)^2+vrr1(2)^2+vrr1(3)^2)+...
       0.5*ml*(vrr2(1)^2+vrr2(2)^2+vrr2(3)^2)+...
       0.5*ml*(vrr3(1)^2+vrr3(2)^2+vrr3(3)^2);

KE = KE1+KE2+KEll+KElr+KErl+KErr;
s = [q;dq];
q_act = [all1;all2;all3;alr1;alr2;alr3;arl1;arl2;arl3;arr1;arr2;arr3] ; %actuated coordinates
[D, C, G, B] = LagrangianDynamics(KE, PE, q, dq, q_act);
%%

matlabFunction(D, 'File', 'gen/Massmat', 'Vars', {q}, 'optimize', false);
matlabFunction(C, 'File', 'gen/Corvec', 'Vars', {s}, 'optimize', false);
matlabFunction(G, 'File', 'gen/Gravvec', 'Vars', {q}, 'optimize', false);
matlabFunction(B, 'File', 'gen/Torquemat', 'Vars', {}, 'optimize', false);

%% Contact point jacobians
Jll = jacobian(pllst,q);
Jlr = jacobian(plrst,q);
Jrl = jacobian(prlst,q);
Jrr = jacobian(prrst,q);
%% Their derivates
dJll = sym(zeros(size(Jll)));
dJlr = sym(zeros(size(Jll)));
dJrl = sym(zeros(size(Jll)));
dJrr = sym(zeros(size(Jll)));
for i = 1:size(Jll, 1)
    for j = 1:size(Jll, 2)
        dJll(i, j) = simplify(jacobian(Jll(i, j), q)*dq);
        dJlr(i, j) = simplify(jacobian(Jlr(i, j), q)*dq);
        dJrl(i, j) = simplify(jacobian(Jrl(i, j), q)*dq);
        dJrr(i, j) = simplify(jacobian(Jrr(i, j), q)*dq);
    end
end
matlabFunction(pll3, 'File', 'gen/footll', 'Vars', {q}, 'optimize', false);
matlabFunction(plr3, 'File', 'gen/footlr', 'Vars', {q}, 'optimize', false);
matlabFunction(prl3, 'File', 'gen/footrl', 'Vars', {q}, 'optimize', false);
matlabFunction(prr3, 'File', 'gen/footrr', 'Vars', {q}, 'optimize', false);
matlabFunction(Jll, 'File', 'gen/footjacll', 'Vars', {q}, 'optimize', false);
matlabFunction(Jlr, 'File', 'gen/footjaclr', 'Vars', {q}, 'optimize', false);
matlabFunction(Jrl, 'File', 'gen/footjacrl', 'Vars', {q}, 'optimize', false);
matlabFunction(Jrr, 'File', 'gen/footjacrr', 'Vars', {q}, 'optimize', false);
matlabFunction(dJll, 'File', 'gen/footdjacll', 'Vars', {s}, 'optimize', false);
matlabFunction(dJlr, 'File', 'gen/footdjaclr', 'Vars', {s}, 'optimize', false);
matlabFunction(dJrl, 'File', 'gen/footdjacrl', 'Vars', {s}, 'optimize', false);
matlabFunction(dJrr, 'File', 'gen/footdjacrr', 'Vars', {s}, 'optimize', false);
%% ODE solver
tic
x0 = [0;0;-(2-sqrt(2))*0.5;0;0;0;0;0;0;0;pi/4;pi/2;0;-pi/4;-pi/2;0;pi/4;pi/2;0;-pi/4;-pi/2;zeros(21,1)];
tspan = linspace(0,30,1000);
[t,x] = ode45(@centipede_dynamics,tspan,x0);
close all
figure()
plot(t,x(:,1:3))
figure()
plot(t,x(:,4:6))
figure()
plot(t,x(:,7:9))
figure()
plot(t,x(:,10:12))
figure()
plot(t,x(:,13:15))
figure()
plot(t,x(:,16:18))
figure()
plot(t,x(:,19:21))
figure()
plot(t,x(:,22:42))
toc
