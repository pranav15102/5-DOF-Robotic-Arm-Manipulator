function [base, feeder] = constructToolStation(globalPose, showResults)

    if nargin < 2
        showResults = false;
    end

    % Create the base
    base = collisionBox(.1, .1, .4);
    base.Pose = trvec2tform([0 0 base.Z/2]);

    % Add the part feeder and position relative to base
    feeder = collisionBox(.1, .15, .01);
    feeder.Pose = trvec2tform([0 0 base.Pose(3,4)+base.Z/2+feeder.Y/2])*axang2tform([1 0 0 pi/4]);

    % Position all the pieces globally
    base.Pose = globalPose*base.Pose;
    feeder.Pose = globalPose*feeder.Pose;

    % Show results
    if showResults
        figure();
        ax = gca;
        base.show('Parent', ax);
        hold all
        feeder.show('Parent', ax);
        feeder2.show('Parent', ax);
    end
end