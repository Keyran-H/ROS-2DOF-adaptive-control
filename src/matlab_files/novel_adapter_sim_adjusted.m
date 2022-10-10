clear
close all

% Define custom parameters
simulation_time = 60; % seconds
timeStep = 0.001; % seconds

% Custom Gains
Kr = 1;
Kv = 15;
Kp = 5;
gamma = 20*eye(5);
Kfilt = 1000;
Kff = 0.001;
Kinit = 0.001;
Komega1 = 0;
Komega2 = 0.01;

% Simulation states
timeIntervals = 0:timeStep:simulation_time;
Phi_m1f_prev = zeros(2,5);
Phi_m2f_prev = zeros(2,5);
Phi_vgf_prev = zeros(2,5);
Phi_f_prev = zeros(2,5);
tau_f_prev = zeros(2,1);
W_t_prev = Kinit*eye(5);
N_t_prev = zeros(5,1);


q_robot = [0; 0]; % Make the robot start at joint position 0
qd_robot = [0;0];

% Initial Estimates
m1 = 2.4;
m2 = 2.9;
l1 = 0.3;
l2 = 0.4;
Izz1 = 0.002;
Izz2 = 0.004;


% m1 = 0.1;
% m2 = 1.7;
% l1 = 0.3;
% l2 = 3;
% Izz1 = 4;
% Izz2 = 2;

% m1 = 1;
% m2 = 1;
% l1 = 1;
% l2 = 1;
% Izz1 = 1;
% Izz2 = 1;

theta_hat = [m1*l1^2 + m2*l1^2 + m2*l2^2 + Izz1 + Izz2; m2*l1*l2; m2*l2^2 + Izz2; m1*l1 + m2*l1; m2*l2]; % Double checked!

theta_hat_data = [];
e_robot_data = [];
ed_robot_data = [];
trajTimes = [];
Phi_f_all = [];
q_robot_data = [];
qd_robot_data = [];
elapsed_time = 0;

% Debug
qdd_robot_debug = [];

for i=timeIntervals

    %%% Get the trajectory
    [qt, qtd, qtdd] = getTrajectoryPt(elapsed_time);

    % Compute errors
    e_robot = qt - q_robot;
    qrd = qtd + Kp*e_robot;
    ed_robot = qtd - qd_robot;
    qrdd = qtdd + Kv*ed_robot;
    r_robot = qrd - qd_robot;

    % Compute control torques
    Phi = GetPhi(q_robot, qd_robot, qrd, qrdd);
    tau = Phi*theta_hat + Kr*r_robot;
    
    % Apply Torque to robot and update the robot state variables
    [M, Vm, G] = getRobotDynamics(q_robot, qd_robot);
    qdd_robot = -1*(M\(Vm*qd_robot + G - tau));
    qd_robot = qd_robot + qdd_robot*timeStep;
    q_robot = q_robot + qd_robot*timeStep;

    % Recompute r and Phi for finding theta_hat (because the robot state changed)
    e_robot = qt - q_robot;
    qrd = qtd + Kp*e_robot;
    ed_robot = qtd - qd_robot;
    qrdd = qtdd + Kv*ed_robot;
    r_robot = qrd - qd_robot;

    Phi = GetPhi(q_robot, qd_robot, qrd, qrdd);

    % Novel Adaptive Controller Adjustment

    % Get current phi matrices
    Phi_m1_curr = GetPhi_m1(q_robot, qd_robot);
    Phi_m2_curr = GetPhi_m2(q_robot, qd_robot);
    Phi_vg_curr = GetPhi_vg(q_robot, qd_robot);

    % Get the filtered phi and tau
    Phi_m1f_curr = getFiltered(Phi_m1_curr, Phi_m1f_prev, Kfilt, timeStep);
    Phi_m2f_curr = getFiltered(Phi_m2_curr, Phi_m2f_prev, Kfilt, timeStep);
    Phi_vgf_curr = getFiltered(Phi_vg_curr, Phi_vgf_prev, Kfilt, timeStep);
    tau_f_curr = getFiltered(tau, tau_f_prev, Kfilt, timeStep);

    % Get Phi_f
    Phi_f_curr = (Phi_m1_curr - Phi_m1f_curr)/Kfilt + Phi_m2f_curr + Phi_vgf_curr;

    % Compute W(t) and N(t)
    W_t_denom = 1 + Kff*timeStep;
    W_t_numer = W_t_prev + Kff*(Phi_f_curr')*(Phi_f_curr)*timeStep;
    W_t_curr = W_t_numer / W_t_denom;

    N_t_denom = 1 + Kff*timeStep;
    N_t_numer = N_t_prev + Kff*(Phi_f_curr')*tau_f_curr*timeStep;
    N_t_curr = N_t_numer / N_t_denom;
  
    % Update the adapted parameters
    denom = (eye(size(W_t_curr)) + timeStep*gamma*Komega2*W_t_curr);
    theta_hat = denom\(theta_hat + timeStep*gamma*Phi'*r_robot + timeStep*gamma*Komega2*N_t_curr);
    theta_hat = theta_hat + gamma*Komega1*sign(W_t_curr*theta_hat - N_t_curr);

    % Save
    theta_hat_data = [theta_hat_data theta_hat];
    e_robot_data = [e_robot_data e_robot];
    ed_robot_data = [ed_robot_data ed_robot];
    trajTimes = [trajTimes elapsed_time];
    q_robot_data = [q_robot_data q_robot ];
    qd_robot_data = [qd_robot_data qd_robot];
    
    % Update
    elapsed_time = elapsed_time + timeStep;
    Phi_m1f_prev = Phi_m1f_curr;
    Phi_m2f_prev = Phi_m2f_curr;
    Phi_vgf_prev = Phi_vgf_curr;
    W_t_prev = W_t_curr;
    
end

q_robot_matnovel_data = q_robot_data;
qd_robot_matnovel_data = qd_robot_data;
e_robot_matnovel_data = e_robot_data;
ed_robot_matnovel_data = ed_robot_data;
theta_hat_matnovel_data = theta_hat_data;

% % Compute integral absolute error
Error_abs_int = sum(abs(e_robot_matnovel_data),2)./simulation_time

% figure
% plot(timeIntervals, q_robot_data(1,:), 'LineWidth', 2, 'LineStyle', '-')
% hold on
% plot(timeIntervals, qt(1,:), 'LineWidth', 2.5)
% xlabel('time')
% ylabel('position')
% title('joint1 trajectory tracking')
% grid on
% 
% figure
% plot(timeIntervals, q_robot_data(2,:), 'LineWidth', 2, 'LineStyle', '-')
% hold on
% plot(timeIntervals, qt(2,:), 'LineWidth', 2.5)
% xlabel('time')
% ylabel('position')
% title('joint2 trajectory tracking')

figure
plot(trajTimes,theta_hat_data)
title('Parameter Convergence')
xlabel('time')
legend('theta 1', 'theta 2', 'theta 3', 'theta 4', 'theta 5')


figure
plot(trajTimes,e_robot_data)
title('Position Error')
xlabel('time')
ylabel('position error / rad')
legend('joint1', 'joint2')


figure
plot(trajTimes,ed_robot_data)
title('Velocity Errors')
legend('joint1', 'joint2')
xlabel('time')
ylabel('velocity error')


function curr_phi_f = getFiltered(curr_phi, prev_phi_f, Kfilt, timeStep)
    curr_phi_f = (timeStep*curr_phi + Kfilt*prev_phi_f)/(Kfilt + timeStep);
end


function [qt, qtd, qtdd] = getTrajectoryPt(t)
    qt = [deg2rad(17.4534 + sin(0.1*t + 2) + 16*sin(0.2*t + 10) + 18*sin(0.3*t + 12));... % t=0 -> -17.4534
          deg2rad(0.8189 + 8*sin(0.2*t+2) + 6*sin(0.3*t+10) + 9*sin(0.36*t+12))];        % t=0 -> -0.8189
    
    qtd = [deg2rad(cos(t/10 + 2)/10 + (16*cos(t/5 + 10))/5 + (27*cos((3*t)/10 + 12))/5);...
           deg2rad((8*cos(t/5 + 2))/5 + (9*cos((3*t)/10 + 10))/5 + (81*cos((9*t)/25 + 12))/25)];
    
    qtdd = [deg2rad(sin(t/10 + 2)/100 - (16*sin(t/5 + 10))/25 - (81*sin((3*t)/10 + 12))/50);
            deg2rad((8*sin(t/5 + 2))/25 - (27*sin((3*t)/10 + 10))/50 - (729*sin((9*t)/25 + 12))/625)];    
end

function [M, Vm, G] = getRobotDynamics(q_robot, qd_robot)
    m1 = 2.35;
    m2 = 3.0;
    l1 = 0.2735;
    l2 = 0.44;
    Izz1 = 0.0029375;
    Izz2 = 0.00375;
    g = 9.81;

    M = [m1*l1^2 + m2*(l1^2+l2^2+2*l1*l2*cos(q_robot(2,1)) + Izz1 + Izz2), m2*(l2^2 + l1*l2*cos(q_robot(2,1))) + Izz2; ...
         m2*(l2^2 + l1*l2*cos(q_robot(2,1))) + Izz2,                       m2*l2^2 + Izz2];
    
    h = -1*m2*l1*l2*sin(q_robot(2,1));
    Vm = [h*qd_robot(2,1), h*(qd_robot(1,1) + qd_robot(2,1));...
          -1*h*qd_robot(1,1), 0];

    G = [g*(m1*l1 + m2*l1)*cos(q_robot(1,1)) + m2*l2*g*cos(q_robot(1,1) + q_robot(2,1));...
         m2*l2*g*cos(q_robot(1,1) + q_robot(2,1))];
end

function Phi_vg = GetPhi_vg(q_robot, qd_robot)
    q1 = q_robot(1,1);
    q2 = q_robot(2,1);
    q1d = qd_robot(1,1);
    q2d = qd_robot(2,1);
    g = 9.81;

    Phi_vg = [0, -q1d*sin(q2)*q2d - q2d*sin(q2)*(q2d + q1d), 0, g*cos(q1), g*cos(q1 + q2); ...
          0, sin(q2)*q1d^2, 0, 0, g*cos(q1 + q2)];
end

function Phi_m1 = GetPhi_m1(q_robot, qd_robot)
    q1 = q_robot(1,1);
    q2 = q_robot(2,1);
    q1d = qd_robot(1,1);
    q2d = qd_robot(2,1);

    Phi_m1 = [q1d, 2*cos(q2)*q1d + cos(q2)*q2d, q2d, 0, 0;...
               0, cos(q2)*q1d, q1d + q2d, 0, 0];
end

function Phi_m2 = GetPhi_m2(q_robot, qd_robot)
    q1 = q_robot(1,1);
    q2 = q_robot(2,1);
    q1d = qd_robot(1,1);
    q2d = qd_robot(2,1);

    Phi_m2 = [0, 2*q2d*sin(q2)*q1d + q2d^2*sin(q2), 0, 0, 0;...
              0,  q2d*sin(q2)*q1d, 0, 0, 0];
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
    g = 9.81;

    Phi = [qr1dd, cos(q2)*(2*qr1dd + qr2dd) - sin(q2)*(q2d*qr1d + (q1d + q2d)*qr2d), qr2dd, g*cos(q1), g*cos(q1+q2); ...
           0, qr1dd*cos(q2) + q1d*qr1d*sin(q2), qr1dd + qr2dd, 0, g*cos(q1 + q2)];

end

