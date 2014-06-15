function [rigidErrorModel] = ...
    estimateRigidTransform(cameraPoses, robotPoses, trainingSize)
    
    % set up the training indices and testing indices
    [N p] = size(cameraPoses);
    indices = randperm(N);
    validationSize = 0.0;
    testSize = 1.0 - trainingSize - validationSize;
      
    trainingIndex = round(trainingSize * N);
    validationIndex = trainingIndex + round(validationSize * N);
    trainingIndices = indices(1:trainingIndex);
    validationIndices = indices((trainingIndex+1):validationIndex);
    testingIndices = indices((trainingIndex+1):N);

    % run the regression
    n_dim = 16;
    n_testing = size(testingIndices,2);
      
    training_x = cameraPoses(trainingIndices,:);
    testing_x = cameraPoses(testingIndices,:);
      
    training_y = robotPoses(trainingIndices,:);
    testing_y = robotPoses(testingIndices,:);
    testing_y_predicted = zeros(N, n_dim);

    [R, t] = find_rigid_transformation(training_y, training_x);
    T = [R t; zeros(1,3) 1];

    % apply learned rigid offset
    testing_y_predicted = applyRigidTransform(testing_x, T);

    % evaluate errors
    testPredictedRobotPoses = testing_y_predicted;
    testTrueRobotPoses = testing_y;
    testCameraPoses = testing_x;
    
    testError = struct();
    [testError.matrixError, testError.translationError, testError.rotationError] = ...
        evaluateErrors(testPredictedRobotPoses, testTrueRobotPoses);

    rigidErrorModel = struct();
    rigidErrorModel.testError = testError;
    rigidErrorModel.T = T;
    rigidErrorModel.testPredictedRobotPoses = testPredictedRobotPoses;
    rigidErrorModel.testTrueRobotPoses = testTrueRobotPoses;
    rigidErrorModel.testCameraPoses = testCameraPoses;
end

