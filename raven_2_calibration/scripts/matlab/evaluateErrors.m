function [matrixErrorStruct, translationErrorStruct, rotationErrorStruct] = ...
evaluateErrors(predictedPoses, truePoses)
    N = size(predictedPoses,1);
    
    % errors per matrix entry
    matrixErrorStruct = struct();
    matrixErrorStruct.rawError = predictedPoses - truePoses;
    matrixErrorStruct.absError = abs(matrixErrorStruct.rawError);
    matrixErrorStruct.avgError = mean(matrixErrorStruct.absError, 1);
    matrixErrorStruct.stdError = std(matrixErrorStruct.absError, 1);
    matrixErrorStruct.rmsError = sqrt(sum(matrixErrorStruct.absError.^2) / N);
    
    % compute componentwise errors
    translationErrorStruct = struct();
    rotationErrorStruct = struct();
    
    translationError = zeros(N,3);
    eulerAngularError = zeros(N,3);
    quatError = zeros(N,4);
    quatAngularError = zeros(N,1);
    
    for i = 1:(N-1)
        if i == N
            test = 1;
        end
        cameraPoseMat = reshape(predictedPoses(i,:), 4, 4)';
        robotPoseMat = reshape(truePoses(i,:), 4, 4)';
        deltaPoseMat = inv(cameraPoseMat) * robotPoseMat;
        cameraRot = cameraPoseMat(1:3,1:3);
        robotRot = robotPoseMat(1:3,1:3);
        deltaRot = deltaPoseMat(1:3,1:3);
        
        %transError = cameraPoseMat(1:3,4) - robotPoseMat(1:3,4);
        transError = deltaPoseMat(1:3,4);
        [cYaw, cPitch, cRoll] = dcm2angle(cameraRot);
        [rYaw, rPitch, rRoll] = dcm2angle(robotRot);
        [dYaw, dPitch, dRoll] = dcm2angle(deltaRot);
        cQuat = dcm2quat(cameraRot);
        rQuat = dcm2quat(robotRot);
        dQuat = dcm2quat(deltaRot);
        if cQuat(4) < 0
            cQuat = -cQuat;
        end
        if rQuat(4) < 0
            rQuat = -rQuat;
        end
        if dQuat(4) < 0
            dQuat = -dQuat;
        end
        %eulerAngleError = [cYaw - rYaw, cPitch - rPitch, cRoll - rRoll]';
        eulerAngleError = [dYaw, dPitch, dRoll]';      
        %quatAngleError = acos(2*(cQuat*rQuat')^2 - 1);
        quatAngleError = acos(2*(dQuat*[0 0 0 1]')^2 - 1);
        
        translationError(i,:) = transError';
        eulerAngularError(i,:) = eulerAngleError';
        quatError(i,:) = (cQuat - rQuat)';
        quatAngularError(i,:) = quatAngleError;
    end
    
    translationErrorStruct.rawError = translationError;
    translationErrorStruct.absError = abs(translationError);
    translationErrorStruct.maxErrorPrctile = prctile(abs(translationError), 95);
    translationErrorStruct.maxError = max(translationErrorStruct.absError);
    translationErrorStruct.medError = median(translationErrorStruct.absError, 1);
    translationErrorStruct.avgError = mean(translationErrorStruct.absError, 1);
    translationErrorStruct.rmsError = sqrt(mean(translationErrorStruct.absError.^2, 1));
    translationErrorStruct.covariance = cov(translationError);
    translationErrorStruct.stdError = std(abs(translationError), 0, 1);
    
    rotationErrorStruct.eulerError = abs(eulerAngularError);
    rotationErrorStruct.maxEulerPrctile = prctile(abs(eulerAngularError), 95);
    rotationErrorStruct.quatError = abs(quatError);
    rotationErrorStruct.eulerCovariance = cov(eulerAngularError);
    rotationErrorStruct.quatCovariance = cov(quatError);
    rotationErrorStruct.quatAngularError = abs(quatAngularError);
    rotationErrorStruct.medEulerError = median(rotationErrorStruct.eulerError, 1);
    rotationErrorStruct.maxEulerError = max(rotationErrorStruct.eulerError);
    rotationErrorStruct.meanEulerError = mean(rotationErrorStruct.eulerError, 1);
    rotationErrorStruct.rmsEulerError = sqrt(mean(rotationErrorStruct.eulerError.^2, 1));
    rotationErrorStruct.stdEulerError = std(abs(eulerAngularError));
    
    rotationErrorStruct.medQuatError = median(rotationErrorStruct.quatError, 1);
    rotationErrorStruct.maxQuatError = max(rotationErrorStruct.quatError);
    rotationErrorStruct.meanQuatError = mean(rotationErrorStruct.quatError, 1);
    rotationErrorStruct.medQuatAngularError = median(rotationErrorStruct.quatAngularError, 1);
    rotationErrorStruct.maxQuatAngularError = max(rotationErrorStruct.quatAngularError);
    rotationErrorStruct.meanQuatAngularError = mean(rotationErrorStruct.quatAngularError, 1);
end

