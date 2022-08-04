%%% CSV Stuff %%%
data = horzcat(qt', qtd', qtdd');
writematrix(data, "trajectory_test_100Hz.csv");


% traj_msg = rosmessage('trajectory_msgs/JointTrajectory');
% traj_msg.JointNames = jointNames;
% traj_msg.Points = [];
% 
% for i=1:length(trajTimes)
%     traj_pt_msg_points = rosmessage('trajectory_msgs/JointTrajectoryPoint');
%     traj_pt_msg_points.Positions = [q(1,i),q(2,i)];
%     traj_pt_msg_points.Velocities = [qd(1,i),qd(2,i)];
%     traj_pt_msg_points.Accelerations = [qdd(1,i),qdd(2,i)];
%     traj_pt_msg_points.TimeFromStart = rosduration(trajTimes(i));
% %     hi = size(traj_msg.Points)
% %     bye = size(traj_pt_msg_points)
%     traj_msg.Points = vertcat(traj_msg.Points, traj_pt_msg_points);
% end
% 
% writestruct(traj_msg,"trajectory_test.csv")
% Kiran, if there's not much time descrepancy between a hacky method you
% need to come up with and this. CHoose this because it is cleaner and your
% code might be more readable. Also less time spent worrying about the
% arrangement of the fields.