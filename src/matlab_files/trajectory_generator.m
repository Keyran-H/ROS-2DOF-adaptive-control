clear
close all

% Import the Robot
robot = importrobot("/home/kiran/dissertation/ros_experimenting_ws/src/ros_experimenting/models/planar_RR_robot.urdf");
endEffector = "planar_RR_link3";
timeStep = 0.001; % seconds
toolSpeed = 0.1; % m/s
showTrajectory = false;


currentRobotJConfig = homeConfiguration(robot);
numJoints = numel(currentRobotJConfig);

initTime = 0;
jointInit = currentRobotJConfig;
taskInit = getTransform(robot,jointInit,endEffector);
taskFinal = trvec2tform([0.899, 0.200, 1.990])*eul2tform([pi 0 pi]);
distance = norm(tform2trvec(taskInit)-tform2trvec(taskFinal));

finalTime = (distance/toolSpeed) - initTime;
trajTimes = initTime:timeStep:finalTime;
timeInterval = [trajTimes(1); trajTimes(end)];

% Generate Joint-Space Trajectory
ik = inverseKinematics('RigidBodyTree',robot);
ik.SolverParameters.AllowRandomRestart = false;
weights = [1 1 1 1 1 1];

initialGuess = jointInit;
jointFinal = ik(endEffector,taskFinal,weights,initialGuess);

% Extract the joint positions and wrap them
jointInitPos = [];
jointFinalPos = [];

for i = 1:numJoints
    jointInitPos = [jointInitPos wrapToPi(jointInit(i).JointPosition)];
    jointFinalPos = [jointFinalPos wrapToPi(jointFinal(i).JointPosition)];

end
jointNames = ["planar_RR_joint1","planar_RR_joint2"]; % TODO: Do this programattically
% Generate the trajectory
ctrlpoints = [jointInitPos',jointFinalPos'];
[qt,qtd,qtdd,pp] = cubicpolytraj(ctrlpoints,timeInterval,trajTimes);
% [q,qd,qdd,pp] = bsplinepolytraj(jointConfigArray,timeInterval,1); % TODO:
% Use this if more than 2DOF are used.

if showTrajectory
    % Return to initial configuration
    show(robot,currentRobotJConfig,'PreservePlot',false,'Frames','off');
    hold on
    for i=1:length(trajTimes)
        for j=1:numJoints
            configNow(j).JointPosition = qt(j,i);% 1x2 matrix
            configNow(j).JointName = jointNames(j);
        end
    
        poseNow = getTransform(robot,configNow,endEffector);
        show(robot,configNow,'PreservePlot',false);
        taskSpaceMarker = plot3(poseNow(1,4),poseNow(2,4),poseNow(3,4),'b.','MarkerSize',20);
        drawnow;
    end
    % Add a legend and title
    title('Manipulator Trajectory')
    legend('Trajectory Points')
end

