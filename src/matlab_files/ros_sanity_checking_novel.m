clear
close all
% sim_states = readtable("/home/kiran/dissertation/ros_experimenting_ws/src/matlab_files/data/trial.csv");
sim_states = readtable("/home/kiran/dissertation/ros_experimenting_ws/src/matlab_files/data/novel_adaptive_data.csv");

sim_states = table2array(sim_states);
period = 0.001; % cell per seconds
desired_plot_time = 0.02;
cells = desired_plot_time / period;

% Custom Gains. REMEMBER TO DOUBLE CHECK THIS
Kr = 0.1;
Kv = 0.1;
Kp = 0.1;
gamma = 0.1*eye(5);

% Initial Estimates
m1 = 1;
m2 = 1;
l1 = 1;
l2 = 1;
Izz1 = 1;
Izz2 = 1;
theta_hat = [m1*l1^2 + m2*l1^2 + m2*l2^2 + Izz1 + Izz2; m2*l1*l2; m2*l2^2 + Izz2; m1*l1 + m2*l1; m2*l2]; % Double checked!

theta_hat_data = [];
e_robot_data = [];
ed_robot_data = [];
etau_robot_data = [];

% Debug
qdd_robot_debug = [];

sim_iterations = size(sim_states);
for i=1:sim_iterations(1)

    % Robot State variables
    q_robot = [sim_states(i,3); sim_states(i,4)];
    qd_robot = [sim_states(i,5); sim_states(i,6)];

    % Trajectory
    qt = [sim_states(i,7); sim_states(i,8)];
    qtd = [sim_states(i,9); sim_states(i,10)];
    qtdd = [sim_states(i,11); sim_states(i,12)];

    % Compute errors
    e_robot = qt - q_robot;
    qrd = qtd + Kp*e_robot;
    ed_robot = qtd - qd_robot;
    qrdd = qtdd + Kv*ed_robot;
    r_robot = qrd - qd_robot;

    % Get Parameters
    theta_hat = [sim_states(i,13); sim_states(i,14); sim_states(i,15); sim_states(i,16); sim_states(i,17)];

    % Compute control torques
    Phi = GetPhi(q_robot, qd_robot, qrd, qrdd);
    matlab_tau = Phi*theta_hat + Kr*r_robot;
    ros_tau = [sim_states(i,18); sim_states(i,19)];

    etau = abs(ros_tau - matlab_tau); 

    % Save
    theta_hat_data = [theta_hat_data theta_hat];
    e_robot_data = [e_robot_data e_robot];
    ed_robot_data = [ed_robot_data ed_robot];
    etau_robot_data = [etau_robot_data etau];

end

% figure
% bar(sim_states(1:cells,1),sim_states(1:cells,2))
mean(sim_states(:,2))

figure
plot(sim_states(:,1),sim_states(:,3))
hold on
plot(sim_states(:,1),sim_states(:,7))
xlabel('time')
ylabel('position')
legend('actual', 'desired')
title('Joint 1 trajectory, desired vs actual')

figure
plot(sim_states(:,1),sim_states(:,4))
hold on
plot(sim_states(:,1),sim_states(:,8))
xlabel('time')
ylabel('position')
legend('actual', 'desired')
title('Joint 2 trajectory, desired vs actual')

figure
plot(sim_states(:,1),e_robot_data)
xlabel('time')
ylabel('position error')
legend('joint1', 'joint2')
title('Position Error')

figure
plot(sim_states(:,1),ed_robot_data)
xlabel('time')
ylabel('velocity error')
legend('joint1', 'joint2')
title('Velocity Errors')

figure
plot(sim_states(:,1),theta_hat_data)
xlabel('time')
ylabel('Parameter')
title('Parameter Convergence')

figure
plot(sim_states(:,1),etau_robot_data)
xlabel('time')
ylabel('Torque mismatch error')
legend('joint1', 'joint2')
title('Torque MATLAB and Sim mismatch')



function [M, Vm, G] = getRobotDynamics(q_robot, qd_robot)
    m1 = 1;
    m2 = 1;
    l1 = 1;
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

function Phi = GetPhi(q_robot, qd_robot, qrd, qrdd)
    q1 = q_robot(1,1);
    q2 = q_robot(2,1);
    q1d = qd_robot(1,1);
    q2d = qd_robot(2,1);
    qr1d = qrd(1,1);
    qr2d = qrd(2,1);
    qr1dd = qrdd(1,1);
    qr2dd = qrdd(2,1);
    g_const = 9.81;

    Phi = [qr1dd, cos(q2)*(2*qr1dd + qr2dd) - sin(q2)*(q2d*qr1d + (q1d + q2d)*qr2d), qr2dd, g_const*cos(q1), g_const*cos(q1+q2); ...
           0, qr1dd*cos(q2) + q1d*qr1d*sin(q2), qr1dd + qr2dd, 0, g_const*cos(q1 + q2)];

end