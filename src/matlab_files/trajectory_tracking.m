% Run trajectory_generator.m first!
% Stable parameters:
% timeStep = 0.01; % seconds
% toolSpeed = 0.1; % m/s

% Custom Gains
Kr = 1;
Kv = 1;
Kp = 0.1;
gamma = 1*eye(5);

% Robot State variables
q_robot = [0;0];
qd_robot = [0;0];

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
q_robot_data = [];
qd_robot_data = [];

% Debug
qdd_robot_debug = [];

for i=1:length(trajTimes)

    % Compute errors
    e_robot = qt(:,i) - q_robot;
    qrd = qtd(:,i) + Kp*e_robot;
    ed_robot = qtd(:,i) - qd_robot;
    qrdd = qtdd(:,i) + Kv*ed_robot;
    r_robot = qrd - qd_robot;

    % Compute control torques
    Phi = GetPhi(q_robot, qd_robot, qrd, qrdd);
    tau = Phi*theta_hat + Kr*r_robot;
    
    % Apply Torque to robot and update the robot state variables
    [M, Vm, G] = getRobotDynamics(q_robot, qd_robot);
    qdd_robot = -1*(M\(Vm*qd_robot + G - tau));
    qd_robot = qd_robot + qdd_robot*timeStep;
    q_robot = q_robot + qd_robot*timeStep;

    % Recompute r and Phi (because the robot state changed)
    e_robot = qt(:,i) - q_robot;
    qrd = qtd(:,i) + Kp*e_robot;
    ed_robot = qtd(:,i) - qd_robot;
    qrdd = qtdd(:,i) + Kv*ed_robot;
    r_robot = qrd - qd_robot;

    Phi = GetPhi(q_robot, qd_robot, qrd, qrdd);

    % Update the adapted parameters
    theta_hat_d = gamma*Phi'*r_robot;
    theta_hat = theta_hat + theta_hat_d*timeStep;

    % Save
    theta_hat_data = [theta_hat_data theta_hat];
    e_robot_data = [e_robot_data e_robot];
    ed_robot_data = [ed_robot_data ed_robot];
    q_robot_data = [q_robot_data q_robot];
    qd_robot_data = [qd_robot_data qd_robot];

    qdd_robot_debug = [qdd_robot_debug qdd_robot];

end

figure
plot(trajTimes, q_robot_data(1,:), 'LineWidth', 2, 'LineStyle', '-')
hold on
plot(trajTimes, qt(1,:), 'LineWidth', 2.5)
xlabel('time')
ylabel('position')
legend('Actual Path', 'Trajectory Demand', 'Location', 'best')
title('planar\_RR\_joint1 trajectory tracking')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_sim/joint1_traj_track.eps', '-depsc')

figure
plot(trajTimes, q_robot_data(2,:), 'LineWidth', 2, 'LineStyle', '-')
hold on
plot(trajTimes, qt(2,:), 'LineWidth', 2.5)
xlabel('time')
ylabel('position')
legend('Actual Path', 'Trajectory Demand', 'Location', 'best')
title('planar\_RR\_joint2 trajectory tracking')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_sim/joint2_traj_track.eps', '-depsc')

figure
plot(trajTimes, e_robot_data, 'LineWidth', 2)
title('Joint Position Errors')
xlabel('time, seconds')
ylabel('Position error, radians')
legend('planar\_RR\_joint1', 'planar\_RR\_joint2', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_sim/joints_pos_errors.eps', '-depsc')

figure
plot(trajTimes,ed_robot_data, 'LineWidth', 2)
title('Joint Velocity Errors')
xlabel('time, seconds')
ylabel('Velocity error, radians')
legend('planar\_RR\_joint1', 'planar\_RR\_joint2', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_sim/joints_vel_errors.eps', '-depsc')

figure
plot(trajTimes,theta_hat_data, 'LineWidth', 2)
title('Parameter Convergence')
xlabel('time, seconds')
legend('theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_sim/parameter_errors.eps', '-depsc')

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

