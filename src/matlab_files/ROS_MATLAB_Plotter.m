% Planar Robot ROS and MATLAB Plotter
% comment the "clear' from the trajectory_generator, trajectory_tracking
% and ros_sanity_checking matlab files. Run in that order
close all

% Trajectory tracking joint 1
figure
plot(trajTimes, qt(1,:), 'LineWidth', 2.5)
hold on
plot(trajTimes, sim_states(:,3), 'LineWidth', 2, 'LineStyle', ':')
hold on
plot(trajTimes, q_robot_data(1,:), 'LineWidth', 2, 'LineStyle', ':')
xlabel('time, seconds')
ylabel('Joint Position, radians')
legend('Trajectory Demand', 'ROS-Gazebo sim','MATLAB sim', 'Location', 'best')
title('planar\_RR\_joint1 Trajectory Tracking')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_ROS_sim/joint1_traj_track.eps', '-depsc')

% Trajectory tracking joint 2
figure
plot(trajTimes, qt(2,:), 'LineWidth', 2.5)
hold on
plot(trajTimes, sim_states(:,4), 'LineWidth', 2, 'LineStyle', ':')
hold on
plot(trajTimes, q_robot_data(2,:), 'LineWidth', 2, 'LineStyle', ':')
xlabel('time, seconds')
ylabel('Joint Position, radians')
legend('Trajectory Demand', 'ROS-Gazebo sim','MATLAB sim', 'Location', 'best')
title('planar\_RR\_joint2 Trajectory Tracking')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_ROS_sim/joint2_traj_track.eps', '-depsc')

% Trajectory ROS joint position error
figure
plot(trajTimes, e_robot_ros_data, 'LineWidth', 2)
title('ROS-Gazebo Joint Position Errors')
xlabel('time, seconds')
ylabel('Position error, radians')
legend('planar\_RR\_joint1', 'planar\_RR\_joint2', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_ROS_sim/joints_pos_errors_ros.eps', '-depsc')

% Trajectory MATLAB joint position error
figure
plot(trajTimes, e_robot_matlab_data, 'LineWidth', 2)
title('MATLAB Joint Position Errors')
xlabel('time, seconds')
ylabel('Position error, radians')
legend('planar\_RR\_joint1', 'planar\_RR\_joint2', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_ROS_sim/joints_pos_errors_matlab.eps', '-depsc')

% Trajectory ROS joint velocity error
figure
plot(trajTimes, ed_robot_ros_data, 'LineWidth', 2)
title('ROS-Gazebo Joint Velocity Errors')
xlabel('time, seconds')
ylabel('Velocity error, radians/s')
legend('planar\_RR\_joint1', 'planar\_RR\_joint2', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_ROS_sim/joints_vel_errors_ros.eps', '-depsc')

% Trajectory MATLAB joint velocity error
figure
plot(trajTimes, ed_robot_matlab_data, 'LineWidth', 2)
title('MATLAB Joint Velocity Errors')
xlabel('time, seconds')
ylabel('Velocity error, radians/s')
legend('planar\_RR\_joint1', 'planar\_RR\_joint2', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_ROS_sim/joints_vel_errors_matlab.eps', '-depsc')

% Parameters ROS
figure
plot(trajTimes,theta_hat_ros_data, 'LineWidth', 2)
title('ROS-Gazebo Parameter Estimation')
xlabel('time, seconds')
legend('theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_ROS_sim/parameter_errors_ros.eps', '-depsc')

% Parameters MATLAB
figure
plot(trajTimes,theta_hat_matlab_data, 'LineWidth', 2)
title('MATLAB Parameter Estimation')
xlabel('time, seconds')
legend('theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'Location', 'best')
grid on
print('~/dissertation/ros_experimenting_ws/src/matlab_files/data/Graphs/MATLAB_Sim_Method1/matlab_ROS_sim/parameter_errors_matlab.eps', '-depsc')