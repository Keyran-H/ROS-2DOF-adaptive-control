% Custom Gains
Kr = 5;
Kv = 1;
Kp = 1;
gamma = 1*eye(5);

% Robot State variables
q_robot = [0;0];
qd_robot = [0;0];

% Initial Estimates
m1 = 2;
m2 = 2;
l1 = 2;
l2 = 2;
Izz1 = 2;
Izz2 = 2;
Theta_hat = [m1*l1^2 + m2*l1^2 + m2*l2^2 + Izz1 + Izz2; m2*l1*l2; m2*l2^2 + Izz2; m1*l1 + m2*l1; m2*l2]; % Double checked!

Theta_hat_data = [];
e_robot_data = [];
ed_robot_data = [];

for i=1:length(trajTimes)

    % Compute errors
    e_robot = qt(:,i) - q_robot;
    qrd = qtd(:,i) + Kp*e_robot;
    ed_robot = qtd(:,i) - qd_robot;
    qrdd = qtdd(:,i) + Kv*ed_robot;
    r_robot = qrd - qtd(:,i);

    % Compute control torques
    Phi = getPhi(q_robot(1,1), q_robot(2,1), qd_robot(1,1), qd_robot(2,1), qrd(1,1), qrd(2,1), qrdd(1,1), qrdd(2,1));
    tau = Phi*Theta_hat + Kr*r_robot;
    
    % Update the robot state variables
    [M, Vm, G] = getRobotDynamics(q_robot, qd_robot);
    qdd_robot = -1*M\(Vm*qd_robot + G - tau);
    qd_robot = qd_robot + qdd_robot*timeStep;
    q_robot = q_robot + qd_robot*timeStep;

    % Update the adapted parameters
    theta_d_hat = gamma*Phi'*r_robot;
    Theta_hat = Theta_hat + theta_d_hat*timeStep;

    % Save
    Theta_hat_data = [Theta_hat_data Theta_hat];
    e_robot_data = [e_robot_data e_robot];
    ed_robot_data = [ed_robot_data ed_robot];

end

figure
title('Positin Error')
plot(trajTimes,e_robot_data)
xlabel('time')
ylabel('position error')

figure
title('Velocity Errors')
plot(trajTimes,ed_robot_data)

xlabel('time')
ylabel('velocity error')

figure
title('Parameter Convergence')
plot(trajTimes,Theta_hat_data)

xlabel('time')



function [M, Vm, G] = getRobotDynamics(q_robot, qd_robot)
    m1 = 1;
    m2 = 1;
    l1 = 1.5;
    l2 = 1;
    Izz1 = 1;
    Izz2 = 1;
    g_const = 9.81;

    M = [m1*l1^2 + m2*(l1^2+l2^2+2*l1*l2*cos(q_robot(2,1)) + Izz1 + Izz2), m2*(l2^2 + l1*l2*cos(q_robot(2,1))) + Izz2; ...
         m2*(l2^2 + l1*l2*cos(q_robot(2,1))) + Izz2,                       m2*l2^2 + Izz2];
    
    h = -1*m2*l1*l2*sin(q_robot(2,1));
    Vm = [h*qd_robot(2,1), h*(qd_robot(1,1) + qd_robot(2,1));...
          -1*h*qd_robot(1,1), 0];

    G = [g_const*(m1*l1 + m2*l1)*cos(q_robot(1,1)) + m2*l2*g_const*cos(q_robot(1,1) + q_robot(2,1));...
         m2*l2*g_const*cos(q_robot(1,1) + q_robot(2,1))];
end

function phi = getPhi(q1, q2, q1d, q2d, qr1d, qr2d, qr1dd, qr2dd)
    g_const = 9.81;
    phi = [qr1dd, cos(q2)*(2*qr1dd + qr2dd) - sin(q2)*(q2d*qr1d + (q1d + q2d)*qr2d), qr2dd, g_const*cos(q1), g_const*cos(q1+q2); ...
           0, qr1dd*cos(q2) + q1d*qr1d*sin(q2), qr1dd + qr2dd, 0, g_const*cos(q1 + q2)];
end

% function theta = getTheta(m1, m2, l1, l2, Izz1, Izz2)
%     theta = [m1*l1^2 + m2*l1^2 + m2*l2^2 + Izz1 + Izz2; m2*l1*l2; m2*l2^2 + Izz2; m1*l1 + m2*l1; m2*l2];
% end