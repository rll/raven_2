function [sysPredictedPoses] = applyRigidTransform(cameraPoses, T)
    N = size(cameraPoses,1);
    sysPredictedPoses = zeros(size(cameraPoses));
    
    for i = 1:N
        pose = reshape(cameraPoses(i,:), 4, 4)';
        predPose = T*pose;
        sysPredictedPoses(i,:) = reshape(predPose', 1, 16);
    end
end

