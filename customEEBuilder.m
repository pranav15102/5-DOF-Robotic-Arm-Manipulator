classdef customEEBuilder < robotics.manip.internal.InternalAccess

    properties (Constant)
        ADHESIVEDIM = [.1 .15 0.001]
        EEDIM = [.01 .02]
    end

    methods (Static)

        function gripper = build(includeAdhesiveObj)

            if nargin < 1
                includeAdhesiveObj = false;
            end

             % Create a rigid body tree to build the gripper off of
            gripper = rigidBodyTree();
            
            % Create the main fixed body, which has several parts to the
            % base. This piece consists of an L-bracket, a box that
            % contains some actuation components, and the mount for the
            % suction element. Start by defining the primitives that will
            % be used to construct the visuals and collisions.
            [gripperBase, suctionPoseAttachments] = customEEBuilder.buildBaseBody();
            gripper.addBody(gripperBase, gripper.BaseName);

            % Add a suction end effector at each attachment point
            suctionEE = customEEBuilder.buildSuctionEE("ee", suctionPoseAttachments{1});
            gripper.addBody(suctionEE, gripperBase.Name);

            % If this gripper includes the adhesive model, attach it to
            % the actuator for modeling purposes
            if includeAdhesiveObj
                adhesiveBody = customEEBuilder.buildAdhesiveBody("adhesiveStrip");
                gripper.addBody(adhesiveBody, suctionEE.Name);
            end
        end
    end

    methods (Static, Access = private)
        function [gripperBase, suctionPoseAttachments] = buildBaseBody()

            gripperBaseComponents = struct('Type', [], 'Dim', [], 'Tform', []);

            % Top of the L bracket
            gripperBaseComponents(1).Type = "Box";
            gripperBaseComponents(1).Dim = [.08 .08 .005];
            gripperBaseComponents(1).Tform = trvec2tform([0 0 gripperBaseComponents(1).Dim(3)/2]);
            gripperBaseComponents(1).Color = [.7 .7 .7];

            % Side of the L bracket
            gripperBaseComponents(2).Type = "Box";
            gripperBaseComponents(2).Dim = [.08 .08 .005];
            gripperBaseComponents(2).Tform = trvec2tform([0 gripperBaseComponents(2).Dim(2)/2 gripperBaseComponents(1).Dim(3)/2+gripperBaseComponents(1).Dim(2)/2])*axang2tform([1 0 0 pi/2]);
            gripperBaseComponents(2).Color = [.7 .7 .7];

            % Actuator housing
            gripperBaseComponents(3).Type = "Box";
            gripperBaseComponents(3).Dim = [.08 .08 .03];
            gripperBaseComponents(3).Tform = trvec2tform(gripperBaseComponents(2).Tform(1:3,4)')*trvec2tform([0 gripperBaseComponents(2).Dim(2)/2+gripperBaseComponents(2).Dim(3)/2 gripperBaseComponents(2).Dim(1)/2-+gripperBaseComponents(3).Dim(3)/2]);
            gripperBaseComponents(3).Color = [.1 .1 .1];

            % Suction mount
            gripperBaseComponents(4).Type = "Box";
            gripperBaseComponents(4).Dim = [.03 .1 .001];
            gripperBaseComponents(4).Tform = trvec2tform(gripperBaseComponents(3).Tform(1:3,4)')*trvec2tform([0 -gripperBaseComponents(3).Dim(2)/2 gripperBaseComponents(3).Dim(3)/2+gripperBaseComponents(4).Dim(2)/2])*axang2tform([1 0 0 pi/2]);
            gripperBaseComponents(4).Color = [.5 .5 .5];

            % Assemble all the pieces
            gripperBase = rigidBody("gripperBase");
            for i = 1:numel(gripperBaseComponents)
                componentDetails = gripperBaseComponents(i);
                colordata.rgba = componentDetails.Color;
                gripperBase.BodyInternal.addVisualInternal(componentDetails.Type, componentDetails.Dim, componentDetails.Tform, colordata);
                gripperBase.addCollision(componentDetails.Type, componentDetails.Dim, componentDetails.Tform);
            end

            % Make sure the attachment joint is sufficiently offset from
            % the base so that there is no collision
            bodyJoint = rigidBodyJoint("gripperBaseJoint");
            bodyJoint.setFixedTransform(trvec2tform([0 0 gripperBaseComponents(1).Dim(3)/2]));
            gripperBase.Joint = bodyJoint;

            % Second output is the array of attachment points for the
            % suction grippers
            suctionPoseAttachments = {gripperBaseComponents(4).Tform};

        end

        function eeBody = buildSuctionEE(bodyName, attachmentPose)
            %buildSuctionEE Build end effector

            eeBody = rigidBody(bodyName);
            eeBody.BodyInternal.addVisualInternal("Cylinder", customEEBuilder.EEDIM);

            eeBodyJoint = rigidBodyJoint(sprintf("%s_Joint", bodyName), "prismatic");
            attachmentOffset = trvec2tform([0 0 customEEBuilder.EEDIM(2)/2]);
            eeBodyJoint.setFixedTransform(attachmentPose*attachmentOffset);
            eeBodyJoint.PositionLimits = [0 0.02];
            eeBody.Joint = eeBodyJoint;

        end

        function attachmentBody = buildAdhesiveBody(bodyName)

            colordata.rgba = [0 .7 .7];
            attachmentBody = rigidBody(bodyName);
            attachmentBody.BodyInternal.addVisualInternal("Box", customEEBuilder.ADHESIVEDIM, eye(4), colordata);
            attachmentBody.addCollision("Box", customEEBuilder.ADHESIVEDIM);

            bodyJoint = rigidBodyJoint(sprintf("%s_Jt", bodyName));
            bodyJoint.setFixedTransform(trvec2tform([0 0 customEEBuilder.EEDIM(2)/2]));    
            attachmentBody.Joint = bodyJoint;

        end
    end
end