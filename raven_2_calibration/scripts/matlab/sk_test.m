% script to load in trajectories, find constant offset
clear; clc; close all;
%%
cameraPoses = zeros(0,16);
robotPoses = zeros(0,16);
cameraGradient = zeros(0,16);
cameraGradientNorm = zeros(0,1);

trajStartI = 12;
trajEndI = 51;
arm = 'L';

filenameTemplate = 'trajectory_data/left_arm_data_%d_%s.mat';
curI = 1;

for i = trajStartI:trajEndI
    filename = sprintf(filenameTemplate, i, arm);
    load(filename);
    
    N = size(c,1);
    cameraPoses(curI:(curI+N-1),:) = c;
    robotPoses(curI:(curI+N-1),:) = r;
    
    %add gradient calculations
    cameraGradient(curI:(curI+N-1),:) = [zeros(1,16);c(1:end-1,:)]-[zeros(1,16);c(2:end,:)];%Preceding gradient, set intial to zero
    cameraGradientNorm(curI:(curI+N-1),:) = sqrt(sum((cameraGradient(curI:(curI+N-1),:) .* cameraGradient(curI:(curI+N-1),:))')');
    
    curI = curI + N;
end
   
% get initial error
rawDataError = struct();
[rawDataError.matrixError, rawDataError.translationError, rawDataError.rotationError] = ...
        evaluateErrors(cameraPoses, robotPoses);

% learn systematic offset
trainingSize = 0.75;
[sysTestError, T, sysPredictedRobotPoses, sysTrueRobotPoses, sysCameraPoses] = ...
    estimateRigidTransform(cameraPoses, robotPoses, trainingSize);

% apply systematic offset
sysDataError = struct();
sysPredictedRobotPoses = applyRigidTransform(cameraPoses, T);
[sysDataError.matError, sysDataError.transError, sysDataError.rotError] = ...
    evaluateErrors(sysPredictedRobotPoses, robotPoses);
transErrorPerDim = sysDataError.transError.rawError;

%%
% learn residual error
%[gpTestError, gpModel, gpPredictedRobotPoses, gpTrueRobotPoses, gpCameraPoses] = ...
%    estimateResidualModel(sysPredictedRobotPoses, robotPoses, trainingSize);

%% bound residuals


%% gaussian mixtures
n_comp = 10;
AIC = zeros(1,n_comp);
BIC = zeros(1,n_comp);
obj = cell(1,n_comp);
for k = 1:n_comp
    obj{k} = gmdistribution.fit(transErrorPerDim, k, 'CovType','full');
    AIC(k)= obj{k}.AIC;
    BIC(k)= obj{k}.BIC;
end

[minAIC,numComponentsA] = min(AIC);
[minBIC,numComponentsB] = min(BIC);
bestGMM = obj{numComponentsA};

% plot residuals w/ clusters
figure;
scatter3(transErrorPerDim(:,1), transErrorPerDim(:,2), transErrorPerDim(:,3));
hold on;
scatter3(bestGMM.mu(:,1), bestGMM.mu(:,2), bestGMM.mu(:,3), 100.0, 'MarkerEdgeColor', [1 0 0]);
xlabel('X Residual (m)');
ylabel('Y Residual (m)');
zlabel('Z Residual (m)');

%% Gradients
figure;
colorMapGradient = min(cameraGradientNorm/prctile(cameraGradientNorm,98),1);
colorMap = [colorMapGradient zeros(length(colorMapGradient),1) 1-colorMapGradient];
scatter3(transErrorPerDim(:,1), transErrorPerDim(:,2), transErrorPerDim(:,3), 500, colorMap ,'Marker','.');
xlabel('X Residual (m)');
ylabel('Y Residual (m)');
zlabel('Z Residual (m)');

figure;
subplot(1,2,1);
[N p] = size(cameraPoses);
gradientResidualAngles = zeros(N,1);
for i=1:1:N
    gradientResidualAngles(i,1) = acosd(dot(cameraGradient(i,:),sysDataError.matError.rawError(i,:)));
end
hist(gradientResidualAngles,100);
xlabel('Angle Between Gradient And Residual');
subplot(1,2,2);
scatter(gradientResidualAngles,cameraGradientNorm);
xlabel('Angle Between Gradient And Residual');
xlabel('Gradient Norm');

%% conditional pose and error

[N p] = size(cameraPoses);
poseEuler = zeros(N,3);
posePredEuler = zeros(N,3);
poseError = zeros(N,1);
radtodeg = @(x) 180*x/pi;
for i=1:1:N
    poseError(i,1) = norm(sysDataError.matError.rawError(i,:));
    
    cameraPoseMat = reshape(cameraPoses(i,:), 4, 4)';
    cameraRot = cameraPoseMat(1:3,1:3);
    [r1 r2 r3]= dcm2angle(cameraRot);
    
    predMat = reshape(sysPredictedRobotPoses(i,:),4,4)';
    predRot = predMat(1:3,1:3);
     [rr1 rr2 rr3]= dcm2angle(predRot);
    
    poseEuler(i,1) = radtodeg(r1);
    poseEuler(i,2) = radtodeg(r2);
    poseEuler(i,3) = radtodeg(r3);
    
    posePredEuler(i,1) = radtodeg(rr1);
    posePredEuler(i,2) = radtodeg(rr2);
    posePredEuler(i,3) = radtodeg(rr3);
end
colorMapPoseError = min(poseError/prctile(poseError,98),1);
colorMap = [colorMapPoseError zeros(length(colorMapPoseError),1) 1-colorMapPoseError];

figure;
subplot(2,1,1);
scatter3(poseEuler(:,1),poseEuler(:,2),poseEuler(:,3), 500, colorMap ,'Marker','.')
xlabel('Z Rotation');
ylabel('X Rotation');
zlabel('Y Rotation');
title('Error vs. Camera Pose');
subplot(2,1,2);
scatter3(posePredEuler(:,1),posePredEuler(:,2),posePredEuler(:,3), 500, colorMap ,'Marker','.')
xlabel('Z Rotation');
ylabel('X Rotation');
zlabel('Y Rotation');
title('Error vs. Predicted Pose');

%% Euler errors
figure;
T = length(sysTestError.rotationError.eulerError(:,1));
r1 = zeros(T,1);
r2 = zeros(T,1);
r3 = zeros(T,1);
for i=1:1:T
    r1(i) = sysTestError.rotationError.eulerError(i,1);
    r2(i) = sysTestError.rotationError.eulerError(i,2);
    r3(i) = sysTestError.rotationError.eulerError(i,3);
    
    if r1(i)  > pi
        r1(i)  = pi - r1(i) ;
    end
    
    if r2(i)  > pi
        r2(i)  = pi - r2(i) ;
    end
    
    if r3(i)  > pi
        r3(i)  = pi - r3(i) ;
    end
end
scatter3(r1,r2,r3)

%% do the same for the actual poses
n_comp = 10;
AIC = zeros(1,n_comp);
BIC = zeros(1,n_comp);
obj = cell(1,n_comp);
for k = 1:n_comp
    obj{k} = gmdistribution.fit(cameraPoses(:,1:12), k, 'CovType','full');
    AIC(k)= obj{k}.AIC;
    BIC(k)= obj{k}.BIC;
end

[minAIC,numComponentsA] = min(AIC);
[minBIC,numComponentsB] = min(BIC);
bestGMM = obj{numComponentsA};

% reduce dimensionality and plot
cameraPoseTranslations = zeros(N, 3);
gmmAvgs = zeros(k,3);
cameraPoseTranslations(:,1) = cameraPoses(:,4);
cameraPoseTranslations(:,2) = cameraPoses(:,8);
cameraPoseTranslations(:,3) = cameraPoses(:,12);
gmmAvgs(:,1) = bestGMM.mu(:,4);
gmmAvgs(:,2) = bestGMM.mu(:,8);
gmmAvgs(:,3) = bestGMM.mu(:,12);

figure;
scatter3(cameraPoseTranslations(:,1), cameraPoseTranslations(:,2), cameraPoseTranslations(:,3));
hold on;
scatter3(gmmAvgs(:,1), gmmAvgs(:,2), gmmAvgs(:,3), 100.0, 'MarkerEdgeColor', [1 0 0]);
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');

%% pca
transErrorPerDim = gpTestError.translationError.rawError;
[U, mu, vars] = pca(transErrorPerDim');
[Yk, Xhat, avsq] = pcaApply(transErrorPerDim', U, mu, 2);
figure;
scatter(Yk(1,:)', Yk(2,:)'');
hold on;

n_comp = 10;
n_replicates = 40;
options = statset('MaxIter', 500, 'Display', 'off', 'TolFun', 1e-6); 

AIC = zeros(1,n_comp);
BIC = zeros(1,n_comp);
obj = cell(1,n_comp);

for k = 1:n_comp
    fprintf(sprintf('Computing GMM for %d components...\n', k));
    obj{k} = gmdistribution.fit(Yk', k, 'CovType','full', ...
             'Replicates', n_replicates, 'Options', options);
    AIC(k)= obj{k}.AIC;
    BIC(k)= obj{k}.BIC;
end

[minAIC,numComponentsA] = min(AIC);
[minBIC,numComponentsB] = min(BIC);
bestGMM = obj{numComponentsA};

scatter(bestGMM.mu(:,1), bestGMM.mu(:,2), 100.0, 'MarkerEdgeColor', [1 0 0]);

figure;
plot([AIC' BIC']);
