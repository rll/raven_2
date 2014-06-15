
%% compute vector format
cameraPoseVectors = zeros(numPoses, 6);
robotPoseVectors = zeros(numPoses, 6);
cameraGradientVectors = zeros(numPoses, 6);
robotGradientVectors = zeros(numPoses, 6);
for i = 1:(numPoses-1)
    cameraPoseMat = reshape(cameraPoses(i,:), 4, 4)';
    robotPoseMat = reshape(robotPoses(i,:), 4, 4)';
    
    cameraRot = cameraPoseMat(1:3,1:3);
    robotRot = robotPoseMat(1:3,1:3);
    [cYaw, cPitch, cRoll] = dcm2angle(cameraRot);
    [rYaw, rPitch, rRoll] = dcm2angle(robotRot);
    cameraTrans = cameraPoseMat(1:3,4);
    robotTrans = robotPoseMat(1:3,4);
    cameraPoseVectors(i,:) = [cYaw cPitch cRoll cameraTrans'];
    robotPoseVectors(i,:) = [rYaw rPitch rRoll robotTrans'];
    
    if (i > 1)
        cameraGradientVectors(i-1,:) = cameraPoseVectors(i,:) - cameraPoseVectors(i-1,:);
        robotGradientVectors(i-1,:) = robotPoseVectors(i,:) - robotPoseVectors(i-1,:);
    end
end

%[cameraPoses, robotPoses, cameraGradients, robotGradients, num, reject] = ...
%    cleanTrajectories(cameraPoses, robotPoses, cameraGradients, robotGradients);

%%
[i,j] = find(abs(commandPoses(min:max,12)- commandPoses(max,12))< 1e-4);
[ip,jp] = find(abs(cameraPoses(min:max,12)- cameraPoses(max,12))< 1e-4);
diff = ip(1) - i(1);