clear
syms q1 q2 q1d q2d m1 m2 l1 l2 Izz1 Izz2 g

M = [m1*l1^2 + m2*(l1^2 + l2^2 + 2*l1*l2*cos(q2)) + Izz1 + Izz2, m2*(l2^2 + l1*l2*cos(q2)) + Izz2; ...
    m2*(l2^2 + l1*l2*cos(q2)) + Izz2, m2*l2^2 + Izz2];

h = -1*m2*l1*l2*sin(q2);
Vm = [h*q2d, h*(q1d + q2d); ...
      -1*h*q1d, 0];

G = [(m1*l1 + m2*l1)*g*cos(q1) + m2*l2*g*cos(q1 + q2); m2*l2*g*cos(q1 + q2)];

theta_hat = [m1*l1^2 + m2*l1^2 + m2*l2^2 + Izz1 + Izz2; m2*l1*l2; m2*l2^2 + Izz2; m1*l1 + m2*l1; m2*l2]; % Double checked!

dMdq1 = diff(M,q1)*q1d;
dMdq2 = diff(M,q2)*q2d;
Md = dMdq1 + dMdq2;

% Auxilliary Matrices
qd = [q1d; q2d];
f = M*qd;
h1 = -1*Md*qd;
h2 = Vm*qd + G;

% Sanity check the f, h1 and h2. Ensure it aligns with what I already have.
syms q1dd q2dd
qdd = [q1dd; q2dd];
fd = Md*qd + M*qdd;
sanity_tau = fd + h1 + h2;
real_tau = M*qdd + Vm*qd + G;
tau_sanity = isequaln(real_tau,sanity_tau)

% syms th1 th2 th3 th4 th5
% Phi_vg = [0,(-2*q2d*q1d-q2d^2)*sin(q2),0,0,0;...
%           0,-q2d*q1d*sin(q2),0,0,0];
Phi_vg = [0, -q1d*sin(q2)*q2d - q2d*sin(q2)*(q2d + q1d), 0, g*cos(q1), g*cos(q1 + q2); ...
          0, sin(q2)*q1d^2, 0, 0, g*cos(q1 + q2)];
Phi_m2 = [0, 2*q2d*sin(q2)*q1d + q2d^2*sin(q2), 0, 0, 0;...
          0,  q2d*sin(q2)*q1d, 0, 0, 0];
Phi_m1 = [q1d, 2*cos(q2)*q1d + cos(q2)*q2d, q2d, 0, 0;...
           0, cos(q2)*q1d, q1d + q2d, 0, 0];

% Sanity check the Phi matrix for f
sanity_f = Phi_m1*theta_hat;
real_f = f;
f_sanity = isAlways(real_f==sanity_f)

% Sanity check the Phi matrix for h = h1 + h2
sanity_h = Phi_vg*theta_hat + Phi_m2*theta_hat;
real_h = h1 + h2;
h_sanity = isAlways(real_h==sanity_h)

% Sanity check the Phi matrix for h1
sanity_h1 = Phi_m2*theta_hat;
real_h1 = h1;
h1_sanity = isAlways(real_h1==sanity_h1)

% Sanity check the Phi matrix for h2
sanity_h2 = Phi_vg*theta_hat;
real_h2 = h2;
h2_sanity = isAlways(real_h2==sanity_h2)


% Md_test = [-2*theta_hat(2)*q2d*sin(q2), -theta_hat(2)*q2d*sin(q2);...
%             -theta_hat(2)*q2d*sin(q2), 0]
% 
% isAlways(Md==Md_test)

% M_test = [theta_hat(1) + 2*theta_hat(2)*cos(q2), theta_hat(3) + theta_hat(2)*cos(q2);...
%           theta_hat(3) + theta_hat(2)*cos(q2), theta_hat(3)]
% 
% isAlways(M==M_test)

% Vm_test = [-theta_hat(2)*sin(q2)*q2d, -theta_hat(2)*sin(q2)*(q1d + q2d);...
%            theta_hat(2)*sin(q2)*q1d, 0]
% 
% isAlways(Vm==Vm_test)

% G_test = [theta_hat(4)*g*cos(q1) + theta_hat(5)*g*cos(q1 + q2);...
%           theta_hat(5)*g*cos(q1 + q2)]
% 
% isAlways(G==G_test)

% q1d = cos(t/10 + 2)/10 + (16*cos(t/5 + 10))/5 + (27*cos((3*t)/10 + 12))/5
% q2d = (8*cos(t/5 + 2))/5 + (9*cos((3*t)/10 + 10))/5 + (81*cos((9*t)/25 + 12))/25
% 
% q1dd = sin(t/10 + 2)/100 - (16*sin(t/5 + 10))/25 - (81*sin((3*t)/10 + 12))/50
% q2dd = (8*sin(t/5 + 2))/25 - (27*sin((3*t)/10 + 10))/50 - (729*sin((9*t)/25 + 12))/625

