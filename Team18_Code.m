% Create UR5e with Custom End Effector
ur5e_robot = loadrobot("universalUR5e");
%  custom gripper with the customEEBuilder helper function and then
% attach the custom gripper to the robot
customEE = customEEBuilder.build(true);
addSubtree(ur5e_robot,'tool0',customEE);
show(ur5e_robot);
% Create Environment Representation
% Add the feeder station at the position [0 0.4 0]
stationPose = trvec2tform([0 0.4 0]);
[toolStationBase,partFeeder] = constructToolStation(stationPose);
% add the dispensing station, modeled by a small cylinder
% next to the feeder at [.25 .45 .65].
dispensingStation = collisionCylinder(.01,.1);
dispensingStation.Pose = trvec2tform([.25 .45 .65]);
% adding platform underneath the robot located at [0 0 -.011].
platform = collisionBox(1,1,0.02);
platform.Pose = trvec2tform([0 0 -.011]);
% adding box that the robot places adhesive strips on 
% in front of the robot at [0.3 -.3 .05].
box = collisionBox(0.2,0.3,0.1);
box.Pose = trvec2tform([0.3 -.3 .05]);
%Importing the waypoints
inverseKinematicsDesigner("dispensingSessionData.mat")
load pathWaypointData.mat
env = {dispensingStation partFeeder toolStationBase platform box};
%Path Planning between the waypoints
planner = manipulatorRRT(ur5e_robot,env);
planner.MaxConnectionDistance = 0.45;
planner.ValidationDistance = 0.1;
rng(10);
numPts = 25;
numWaypoints = size(pathWaypoints,1);
paths = cell(1,numWaypoints);
for segIdx = 1:numWaypoints
    tic;
    plannedPath = plan(planner, pathWaypoints(segIdx,:), pathWaypoints(mod(segIdx,numWaypoints)+1,:));
    shortenedPath = shorten(planner, plannedPath, 10);
    paths{segIdx} = interpolate(planner, shortenedPath, 10);

    segTime = toc;
    disp(['Done planning for segment ',num2str(segIdx),' in ',num2str(segTime), ' seconds']) %i in %f seconds\n])
end
ax = show(ur5e_robot);
hold all
for i = 1:numel(env)
    env{i}.show("Parent", ax);
end

% Display the figure window the animation
pathFig = ancestor(ax, 'figure');
set(pathFig, "Visible", "on")

% Set up timing and configure robot
r = rateControl(10);
tvec = linspace(1,numWaypoints,numWaypoints*numPts);
ur5e_robot.DataFormat = "row";

% Animate all path segments
for pathSegIdx = 1:numel(paths)
    path = paths{pathSegIdx};

    % For each path segment, step through all the configurations
    for configIdx = 1:size(path,1)
        show(ur5e_robot, path(configIdx,:), "FastUpdate",true, "PreservePlot",false,"Parent",ax);
        waitfor(r);
    end

    % Hold the pose and update the title each time a waypoint is reached
    title(sprintf('Segment %i completed', pathSegIdx), "Parent",ax);
    pause(1);
end
