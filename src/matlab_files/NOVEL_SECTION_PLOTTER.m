% Plot the results for the results for the second methodology section
% clear
close all
% Run grad_desc_adapter_sim.m -> novel_adapter_sim_adjusted.m -> this.

% Should the graphs be categorised based on MATLAB vs ROS OR Grad Desc vs
% Novel? It depends how I'm going to talk about it in the discussion. I
% feel like I will find myself in a better place with the first
% categorisation because this is the order I did things. It may be
% problematic to directly compare a MATLAB and ROS result because one uses
% a physics engine and the other does not.

ros_graddes_states = table2array(readtable("/home/kiran/dissertation/ros_experimenting_ws/src/matlab_files/data/novel_adaptive_data.csv"));
ros_novel_states = table2array(readtable("/home/kiran/dissertation/ros_experimenting_ws/src/matlab_files/data/novel_adaptive_data_paper.csv"));

% Compute integral absolute error
ros_graddes_error_abs_int = sum(abs(ros_graddes_states(:,3:4))',2)./simulation_time
ros_novel_error_abs_int = sum(abs(ros_novel_states(:,3:4))',2)./simulation_time

% Get the axis times
axistimes = ros_graddes_states(:,1) - ros_graddes_states(1,1);

% Joint 1 trajectory
figure 
plot(axistimes, ros_graddes_states(:,7), 'LineWidth', 2.5)              % Trajectory Demand
hold on
plot(axistimes, q_robot_matgrad_data(1,:), 'LineWidth', 2, 'LineStyle', ':') % MATLAB
hold on
plot(axistimes, q_robot_matnovel_data(1,:), 'LineWidth', 2, 'LineStyle', ':') % MATLAB
hold on
plot(axistimes, ros_graddes_states(:,3), 'LineWidth', 2, 'LineStyle', ':')
hold on
plot(axistimes, ros_novel_states(:,3), 'LineWidth', 2, 'LineStyle', ':')   
xlabel('time, seconds')
ylabel('Joint Position, radians')
legend('Trajectory Demand', 'Grad Desc. MATLAB','Novel MATLAB', 'Grad Desc. ROS','Novel ROS', 'Location', 'best')
title('BERT2\_joint1 trajectory tracking')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/Method2/joint1_traj_track.eps', '-depsc')

% Joint 2 trajectory
figure 
plot(axistimes, ros_graddes_states(:,8), 'LineWidth', 2.5)              % Trajectory Demand
hold on
plot(axistimes, q_robot_matgrad_data(2,:), 'LineWidth', 2, 'LineStyle', ':') % MATLAB
hold on
plot(axistimes, q_robot_matnovel_data(2,:), 'LineWidth', 2, 'LineStyle', ':') % MATLAB
hold on
plot(axistimes, ros_graddes_states(:,4), 'LineWidth', 2, 'LineStyle', ':')
hold on
plot(axistimes, ros_novel_states(:,4), 'LineWidth', 2, 'LineStyle', ':')   
xlabel('time, seconds')
ylabel('Joint Position, radians')
legend('Trajectory Demand', 'Gradient Descent MATLAB','Novel MATLAB', 'Gradient Descent ROS','Novel ROS', 'Location', 'best')
title('BERT2\_joint2 trajectory tracking')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/Method2/joint2_traj_track.eps', '-depsc')

% MATLAB trajectory joint error
figure
plot(axistimes, e_robot_matgrad_data, 'LineWidth', 2)
hold on
plot(axistimes, e_robot_matnovel_data, 'LineWidth', 2)
title('BERT2 MATLAB Sim Joint Position Errors')
xlabel('time, seconds')
ylabel('Position Error, radians')
legend('Gradient Descent BERT2\_joint1', 'Gradient Descent BERT2\_joint2', 'Novel BERT2\_joint1', 'Novel BERT2\_joint2', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/Method2/joints_pos_errors_matlab.eps', '-depsc')

% ROS trajectory joint error
ros_grad_pos_err_j1 = ros_graddes_states(:,7) - ros_graddes_states(:,3);
ros_novel_pos_err_j1 = ros_novel_states(:,7) - ros_novel_states(:,3);
ros_grad_pos_err_j2 = ros_graddes_states(:,8) - ros_graddes_states(:,4);
ros_novel_pos_err_j2 = ros_novel_states(:,8) - ros_novel_states(:,4);
figure
plot(axistimes, ros_grad_pos_err_j1, 'LineWidth', 2)
hold on
plot(axistimes, ros_grad_pos_err_j2, 'LineWidth', 2)
hold on
plot(axistimes, ros_novel_pos_err_j1, 'LineWidth', 2)
hold on
plot(axistimes, ros_novel_pos_err_j2, 'LineWidth', 2)
title('BERT2 ROS Sim Joint Position Errors')
xlabel('time, seconds')
ylabel('Position Error, radians')
legend('Gradient Descent BERT2\_joint1', 'Gradient Descent BERT2\_joint2', 'Novel BERT2\_joint1', 'Novel BERT2\_joint2', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/Method2/joints_pos_errors_ros.eps', '-depsc')

% MATLAB joint velocity error
figure
plot(axistimes, ed_robot_matgrad_data, 'LineWidth', 2)
hold on
plot(axistimes, ed_robot_matnovel_data, 'LineWidth', 2)
title('BERT2 MATLAB Sim Joint Velocity Errors')
xlabel('time, seconds')
ylabel('Velocity Error, radians/s')
legend('Gradient Descent BERT2\_joint1', 'Gradient Descent BERT2\_joint2', 'Novel BERT2\_joint1', 'Novel BERT2\_joint2', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/Method2/joints_vel_errors_matlab.eps', '-depsc')

% ROS joint velocity error
ros_grad_vel_err_j1 = ros_graddes_states(:,9) - ros_graddes_states(:,5);
ros_novel_vel_err_j1 = ros_novel_states(:,9) - ros_novel_states(:,5);
ros_grad_vel_err_j2 = ros_graddes_states(:,10) - ros_graddes_states(:,6);
ros_novel_vel_err_j2 = ros_novel_states(:,10) - ros_novel_states(:,6);
figure
plot(axistimes, ros_grad_vel_err_j1, 'LineWidth', 2)
hold on
plot(axistimes, ros_grad_vel_err_j2, 'LineWidth', 2)
hold on
plot(axistimes, ros_novel_vel_err_j1, 'LineWidth', 2)
hold on
plot(axistimes, ros_novel_vel_err_j2, 'LineWidth', 2)
title('BERT2 ROS Sim Joint Velocity Errors')
xlabel('time, seconds')
ylabel('Velocity Error, radians/s')
legend('Gradient Descent BERT2\_joint1', 'Gradient Descent BERT2\_joint2', 'Novel BERT2\_joint1', 'Novel BERT2\_joint2', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/Method2/joints_vel_errors_ros.eps', '-depsc')

% Parameters MATLAB Grad Desc
figure
plot(axistimes, theta_hat_matgrad_data, 'LineWidth', 2)
title('MATLAB Gradient Descent Parameter Estimation')
xlabel('time, seconds')
legend('theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/Method2/parameter_errors_grad_matlab.eps', '-depsc')

% Parameters MATLAB Grad Desc
figure
plot(axistimes, theta_hat_matnovel_data, 'LineWidth', 2)
title('MATLAB Novel Algorithm Parameter Estimation')
xlabel('time, seconds')
legend('theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/Method2/parameter_errors_novel_matlab.eps', '-depsc')

% Parameters ROS Novel Algorithm
theta_hat_rosgrad_data = [ros_graddes_states(:,13), ros_graddes_states(:,14), ros_graddes_states(:,15), ros_graddes_states(:,16), ros_graddes_states(:,17)];
figure
plot(axistimes, theta_hat_rosgrad_data, 'LineWidth', 2)
title('ROS Gradient Descent Parameter Estimation')
xlabel('time, seconds')
legend('theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/Method2/parameter_errors_grad_ros.eps', '-depsc')

% Parameters ROS Novel Algorithm
theta_hat_rosnovel_data = [ros_novel_states(:,13), ros_novel_states(:,14), ros_novel_states(:,15), ros_novel_states(:,16), ros_novel_states(:,17)];
figure
plot(axistimes, theta_hat_rosnovel_data, 'LineWidth', 2)
title('ROS Novel Algorithm Parameter Estimation')
xlabel('time, seconds')
legend('theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/Method2/parameter_errors_novel_ros.eps', '-depsc')