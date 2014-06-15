function [error] = calculateResiduals(M, x)
    cameraPoses = x(1:16,:)';
    robotPoses = x(17:32,:)';
    index = 1;
    N = size(cameraPoses,1);
    inliers = zeros(32,0);
    rotationIndices = [1 2 3 5 6 7 9 10 11];
    cameraPoses(:, rotationIndices) = cameraPoses(:, rotationIndices);
    robotPoses(:, rotationIndices) = robotPoses(:, rotationIndices);
    distance_sum = 0;
    for i = 1:N
        predicted = M.T * reshape(cameraPoses(i,:), 4, 4)';
        distance_sum = distance_sum + norm(reshape(predicted', 1, 16) - robotPoses(i,:));
    end
    error = distance_sum/N;
    
end

