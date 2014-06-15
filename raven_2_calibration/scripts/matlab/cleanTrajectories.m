function [cameraPosesClean, robotPosesClean, commandPosesClean, ...
    cameraGradientsClean, robotGradientsClean, commandGradientsClean, N, reject] = ...
    cleanTrajectories(cameraPoses, robotPoses, commandPoses, ...
    cameraGradients, robotGradients, commandGradients)

%60 degree bounds on the axes from the robot poses
%empirically determined limits
yaw_bounds = [45 135];
pitch_bounds = [-45 45];
roll_bounds = [-135 -45];

[N p] = size(cameraPoses);

robotPosesClean = robotPoses;
cameraPosesClean = cameraPoses;
rejectScore = 0;

%remove records that lie outside the bounds
out_of_bounds = [];
radtodeg = @(x) 180*x/pi;
for i=1:1:N
    cameraPoseMat = reshape(cameraPoses(i,:), 4, 4)';
    cameraRot = cameraPoseMat(1:3,1:3);
    [r1 r2 r3]= dcm2angle(cameraRot);
    
    commandPoseMat = reshape(commandPoses(i,:), 4, 4)';
    
    if radtodeg(r1) < yaw_bounds(1) || radtodeg(r1) > yaw_bounds(2) || ...
            radtodeg(r2) < pitch_bounds(1) || radtodeg(r2) > pitch_bounds(2) || ...
            radtodeg(r3) < roll_bounds(1) || radtodeg(r3) > roll_bounds(2)
            out_of_bounds = [out_of_bounds i];    
    elseif norm(commandPoseMat - eye(4)) < 1e-5
        out_of_bounds = [out_of_bounds i];   
    end
end

%remove large gradients
large_gradient = [];
for i = 1:N-1
    cg = norm(cameraPoses(i+1,:) - cameraPoses(i,:));
    rg = norm(robotPoses(i+1,:) - robotPoses(i,:));
    if cg > 0.4 || rg > 0.4
        large_gradient = [large_gradient i+1];
    end
end

%smooth the trajectories

removal = union(large_gradient, out_of_bounds);
reject = (length(removal)/N > .90);%if we removed more than 90% reject
keep = setdiff(1:1:N,removal);
robotPosesClean = robotPoses(keep,:);
cameraPosesClean = cameraPoses(keep,:);
commandPosesClean = commandPoses(keep,:);
robotGradientsClean = robotGradients(keep, :);
cameraGradientsClean = cameraGradients(keep, :);
commandGradientsClean = commandGradients(keep, :);
N = length(keep);
end

