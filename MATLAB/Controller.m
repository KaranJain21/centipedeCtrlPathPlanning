
function tau = Controller(t, s)

    %% Extract generalized coordinates and velocities
    q = s(1 : 21);
    dq = s(22 : 42);
    pll = footll(q);
    plr = footlr(q);
    prl = footrl(q);
    prr = footrr(q);
    Jll = footjacll(q);
    Jlr = footjaclr(q);
    Jrl = footjacrl(q);
    Jrr = footjacrr(q);

    %% [Control #1] zero control

    mass = 260;
    grav = 9.81;
    kp = 00; %proportional constant for position rcom
    kd = 200; %3000 damping constant for velocity rdot com
    
    desstate = [0;0;0;0;0;0;0;0;0;0;pi/2;0;0;-pi/2;0;0;pi/2;0;0;-pi/2;0]; 
    
    % get back to this to make it more precise
    kr = 20; %100 prop const for orientation
    dr = 10; %50 %damping constant for angular velocity
    KR = kr*eye(3); %proportional matrix for torque %change this
    DR = dr*eye(3); %damping matrix for torque
    ea1 = q(4:6); %current euler angles
    ea1dot = dq(4:6);
    ea2 = q(7:9);
    ea2dot = dq(7:9);
    ea1des = desstate(4:6);
    ea2des = desstate(7:9);
    angveldes = [0;0;0];
    % edot_to_w is the transformation matrix which converts
    % rates of euler angles to angular velocities in the inertial frame
    edot_to_w1 = [1 0 -sin(ea1(2));...
                0 cos(ea1(3)) cos(ea1(2))*sin(ea1(3));...
                0 -sin(ea1(3)) cos(ea1(2))*cos(ea1(3))];
    edot_to_w2 = [1 0 -sin(ea2(2));...
                0 cos(ea2(3)) cos(ea2(2))*sin(ea2(3));...
                0 -sin(ea2(3)) cos(ea2(2))*cos(ea2(3))];
            
    angvel1 = edot_to_w1*[ea1dot(3);ea1dot(2);ea1dot(1)] ; %actual angular velocity
    angvel2 = edot_to_w2*[ea2dot(3);ea2dot(2);ea2dot(1)] ; %actual angular velocity
    Rotcur1 = eul2rotm(ea1'); %current orientation matrix
    Rotcur2 = eul2rotm(ea2');
    Rotdes1 = eul2rotm(ea1des'); %desired orientation
    Rotdes2 = eul2rotm(ea2des');
%     Rotdes = eye(3); %desired orientation
    RdtRb1 = Rotdes1'*Rotcur1; %error in orientation
    RdtRb2 = Rotdes2'*Rotcur2;
    rc = q(1:3)+0.5*(Rotcur1*[-0.5;0;0]+Rotcur2*[0.5;0;0]); %center of mass 
    rcdot = dq(1:3)+0.5*(skew(angvel1)*Rotcur1*[-0.5;0;0]+skew(angvel2)*Rotcur2*[0.5;0;0]); %center of mass velocity
    
    rcdes = desstate(1:3)-[0;0;0]; %desired com position
    rcdotdes = [0;0;0]; %desired com velocity
    frdes = -kp*(rc-rcdes)-kd*(rcdot-rcdotdes); 
    forceGAdes = mass*[0;0;grav]+frdes; %desired contact force
    
    quatdes1 = rotm2quat(RdtRb1);
    quatdes2 = rotm2quat(RdtRb2);
    Trdes1 = -2*(quatdes1(1)*eye(3)+skew(quatdes1(2:end)))*KR*quatdes1(2:end)';
    Trdes2 = -2*(quatdes2(1)*eye(3)+skew(quatdes2(2:end)))*KR*quatdes2(2:end)';
    torqueGAdes1 = Rotcur1*(Trdes1-DR*(angvel1-angveldes));
    torqueGAdes2 = Rotcur2*(Trdes2-DR*(angvel2-angveldes));
    wrenchdes = [forceGAdes;torqueGAdes1;torqueGAdes2];

    Graspmap = [eye(3) eye(3) eye(3) eye(3);...
                skew(pll-rc) skew(plr-rc) zeros(3,3) zeros(3,3);...
                zeros(3,3) zeros(3,3) skew(prl-rc) skew(prr-rc)];

    Jmat = [Jll' Jlr' Jrl' Jrr']; %transpose of jacobian 
%% pseudo inverse of G - minimum norm solution of contact forces   
    %test term for joint motion minimization
    kjp = 00;
    kjd = 50;
    q0 = desstate(1:21) ;
    tauprev = -Jmat*pinv(Graspmap)*wrenchdes; 
    % actuator torques
    B1 = Torquemat();
    tau = pinv(B1)*tauprev-kjp*(q(10:21)-q0(10:21)) - kjd*dq(10:21);
%     tau = zeros(12,1);
%% optimization based solution - minimizes the error between desired wrench and applied wrench
%  

%     A = [];
%     b = [];
%     Aeq = [];
%     beq = [];
%     lb = [];
%     ub = [];
%     costfun = @(Fc)(norm(Graspmap*Fc-wrenchdes))^2;
%     nonlcon = @frictioncone;
%     Fc1guess = [0.1;0.1;0.5];
%     Fcguess = [Fc1guess;Fc1guess;Fc1guess;Fc1guess];
%     options = optimoptions('fmincon','Display','none');
%     Fcoptimal = fmincon(costfun,Fcguess,A,b,Aeq,beq,lb,ub,nonlcon,options); %pinv(Graspmap)*wrenchdes;
%     tauprev = -Jmat*Fcoptimal; 
%     tau = tauprev(7:end);
            
            
            

end

function S = skew(s)
S = [0 -s(3) s(2);...
     s(3) 0 -s(1);...
    -s(2) s(1) 0];
end

