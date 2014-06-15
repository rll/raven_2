% script to load in trajectories, find constant offset
%clear; clc; close all;
%% read in data
%trainingSizes = [0.0050, 0.0100, 0.0250, 0.050, 0.075, 0.100, 0.15, 0.2];
%trainingSizes = [0.001, 0.0025, 0.0050, 0.0100, 0.0250, 0.050, 0.075 0.100];
trainingSizes = 0.05;
%sampleRate = 0.025;

D = 1; % subsampling rate
directories = {'data/calibration/left/trajectory_data3'};
commandLatency = 9;
arm = 'L';
standardize = false;
randPerm = true;
clean = false;
diffs = 0;%[0,1,2,3,5,8,10,12,15,20];

%for j = 1:size(diffs,2)
rng(8000);
diff = diffs(1);
[cameraPoses, robotPoses, commandPoses, cameraGradients, robotGradients, commandGradients,...
    targetPoses, trajStartIndices, trajEndIndices] = loadTrajectories(directories, D, commandLatency, clean, diff, standardize);

numPoses = size(cameraPoses,1);

%% CHEAT!!!!!
oldRobotPoses = robotPoses;
oldRobotGradients = robotGradients;
robotPoses = commandPoses;
robotGradients = commandGradients;

%% get initial error
disp('Calculating initial error...');
rawDataError = struct();
[rawDataError.matrixError, rawDataError.translationError, rawDataError.rotationError] = ...
        evaluateErrors(cameraPoses, robotPoses);

%% break up into training and test set
%for i = 1:size(trainingSizes,2);

fprintf('Starting iteration %d\n', i);
trainingSize= trainingSizes(1);  
tic;

testHoldoutPercent = 0.2; % 20% held out data
maxPose = uint32((1.0 - testHoldoutPercent) * numPoses);
closestTraj = 1;
trainingIndex = round(trainingSize * maxPose);
endTrainingIndex = maxPose;

if ~randPerm
    % sample the first trajectories within limits
    for k = 1:size(trajEndIndices,2)
        if trajEndIndices(k) < trainingIndex
            closestTraj = k;
        end
    end
    maxPose = trajEndIndices(closestTraj);
    trainingIndex = round(sampleRate * maxPose);
    %endTrainingIndex = trajEndIndices(closestTraj) / subsample;
    %trainingIndices = (1:endTrainingIndex)*subsample;
end

indices = randperm(maxPose);    
trainingIndices = indices(1:trainingIndex); 
testingIndices = maxPose:numPoses;

% testCameraPoses = cameraPoses(testingIndices,:);
% testCameraGradients = cameraGradients(testingIndices,:);
% testRobotPoses = robotPoses(testingIndices,:);
% testRobotGradients = robotGradients(testingIndices,:);

%% estimate the rigid offset
disp('Estimating rigid offset...');
fitRigidErrorModel = @(poses) (estimateRigidTransform(poses(1:16,:)', ...
                               poses(17:32,:)', 1.0));

degenFn = @(x) (false);
minSamples = 1;
combinedTrainingPoses = [cameraPoses(trainingIndices,:) robotPoses(trainingIndices,:)]';
if clean
    rigidErrorModel = ...
        LMEDS(combinedTrainingPoses, fitRigidErrorModel, ...
            minSamples, @calculateResiduals, 1000);
else
    rigidErrorModel = estimateRigidTransform(cameraPoses(trainingIndices,:), ...
        robotPoses(trainingIndices,:), 1.0);
end

% apply systematic offset
disp('Applying rigid offset...');
sysTestError = struct();
sysTrainingRobotPoses = applyRigidTransform(cameraPoses(trainingIndices,:), rigidErrorModel.T);
sysTestingRobotPoses = applyRigidTransform(testCameraPoses, rigidErrorModel.T);%cameraPoses(testingIndices,:), rigidErrorModel.T);
%
[sysTestError.matError, sysTestError.translationError, sysTestError.rotationError] = ...
    evaluateErrors(sysTestingRobotPoses, testRobotPoses);%robotPoses(testingIndices,:));
disp('Done estimating rigid offset');

% learn residual error
disp('Estimating residual error model...');
num_iter = 20;

cameraPrevGradients = zeros(size(cameraGradients));
cameraPrevGradients(2:size(cameraGradients,1),:) = cameraGradients(1:(size(cameraGradients,1)-1),:);
combinedGrad = [cameraGradients, cameraPrevGradients];

%%
testCameraGradientsInput = testCameraGradients;
if diff == 0
    testCameraGradientsInputs = zeros(size(testCameraGradients));
end
[gpTestError, gpModel, gpPredictedRobotPoses, gpTrueRobotPoses, gpCameraPoses, gpAllPredictedRobotPoses] = ...
    estimateResidualModel(sysTrainingRobotPoses, robotPoses(trainingIndices,:), cameraGradients(trainingIndices,:), ...
    sysTestingRobotPoses, testRobotPoses, testCameraGradients, num_iter); 
%robotPoses(testingIndices,:), cameraGradients(testingIndices,:), num_iter);
stop = toc;
disp('Done estimating residual error');
fprintf(sprintf('Learning took %f sec\n', stop));

%% plot x translation result
translationIndices = [4, 8, 12];

min = 1;
max = 750;

figure;
subplot(3,1,1);
h = plot([cameraPoses(testingIndices(min:max),translationIndices(1)), robotPoses(testingIndices(min:max),translationIndices(1)), ...
    sysTestingRobotPoses(min:max,translationIndices(1)), gpPredictedRobotPoses(min:max,translationIndices(1))]);
[hleg, hobj] = legend('Raw Observed Pose', 'Commanded Pose', 'Fixed Rigid Offset', 'GPR with Velocity and Data Cleaning', 'Location', 'NorthEast');
ylabel('X Position (m)', 'FontSize', 15);
%title('Error Model Predictions for Translation Components', 'FontSize', 15);

textobj = findobj(hobj, 'type', 'text');
set(textobj, 'fontsize', 9);
%set(hleg, 'position', [0.80, 0.85, 0.02, 0.035]);

subplot(3,1,2);
h = plot([cameraPoses(testingIndices(min:max),translationIndices(2)), robotPoses(testingIndices(min:max),translationIndices(2)), ...
    sysTestingRobotPoses(min:max,translationIndices(2)), gpPredictedRobotPoses(min:max,translationIndices(2))]);
ylabel('Y Position (m)', 'FontSize', 15);
%title('Error Model Predictions for Translation Components');

subplot(3,1,3);
h = plot([cameraPoses(testingIndices(min:max),translationIndices(3)), robotPoses(testingIndices(min:max),translationIndices(3)), ...
    sysTestingRobotPoses(min:max,translationIndices(3)), gpPredictedRobotPoses(min:max,translationIndices(3))]);
xlabel('Sequence Number', 'FontSize', 15);
ylabel('Z Position (m)', 'FontSize', 15);
%title('Error Model Predictions for Translation Components');

%% build error model to save
errorModelSave = cell(1,6);
errorModelSave{1} = gpModel.training_x;
errorModelSave{2} = [cameraPoses(testingIndices,:) cameraGradients(testingIndices,:)];
errorModelSave{3} = gpModel.testing_y;
errorModelSave{4} = gpModel.hyp;
errorModelSave{5} = gpModel.alpha;
errorModelSave{6} = rigidErrorModel.T;
save('results/phasespace/errorModel.mat', 'errorModelSave');

%% build gpTestError to save
testErrorSave = cell(1,10);
testErrorSave{1} = trainingSize;
testErrorSave{2} = diff;
testErrorSave{3} = gpTestError.rotationError.rmsEulerError;
testErrorSave{4} = gpTestError;
testErrorSave{5} = stop;
testErrorSave{6} = size(trainingIndices,2);
testErrorSave{7} = closestTraj;
testErrorSave{8} = rawDataError;
testErrorSave{9} = sysTestError;
save(sprintf('results/phasespace/testError/testError%d.mat', j), 'testErrorSave');
%end
%end
