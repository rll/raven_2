%% plotting the raw poses
min = trajStartIndices(47);
max = trajEndIndices(47);
dim = [4 8 12];

delta = 0;
cmdMin = min-delta;
cmdMax = max-delta;

figure;
subplot(3,1,1);
plot([cameraPoses(min:max,dim(1)), robotPoses(min:max,dim(1)), ...
    commandPoses(cmdMin:cmdMax,dim(1)), targetPoses(min:max,dim(1))]);
legend('Raw Camera Pose', 'Robot Pose', 'Command', 'Target');

subplot(3,1,2);
plot([cameraPoses(min:max,dim(2)), robotPoses(min:max,dim(2)), ...
    commandPoses(cmdMin:cmdMax,dim(2)), targetPoses(min:max,dim(2))]);
legend('Raw Camera Pose', 'Robot Pose', 'Command', 'Target');

subplot(3,1,3);
plot([cameraPoses(min:max,dim(3)), robotPoses(min:max,dim(3)), ...
    commandPoses(cmdMin:cmdMax,dim(3)), targetPoses(min:max,dim(3))]);
legend('Raw Camera Pose', 'Robot Pose', 'Command', 'Target');

%% check hysteresis effects
figure;
subplot(2,3,1);
scatter(robotPoses(testingIndices,4), cameraPoses(testingIndices,4));
xlabel('Observed X Position (m)');
ylabel('Desired X Position (m)');
subplot(2,3,2);
scatter(robotPoses(testingIndices,8), cameraPoses(testingIndices,8));
title('Robot vs Raw Phasespace Poses');
xlabel('Observed Y Position (m)');
ylabel('Desired Y Position (m)');
subplot(2,3,3);
scatter(robotPoses(testingIndices,12), cameraPoses(testingIndices,12));
xlabel('Observed Z Position (m)');
ylabel('Desired Z Position (m)');

subplot(2,3,4);
scatter(robotPoses(testingIndices,4), gpPredictedRobotPoses(:,4));
xlabel('Observed X Position (m)');
ylabel('Desired X Position (m)');
subplot(2,3,5);
scatter(robotPoses(testingIndices,8), gpPredictedRobotPoses(:,8));
title('Robot vs GPR Corrected Poses');
xlabel('Observed Y Position (m)');
ylabel('Desired Y Position (m)');
subplot(2,3,6);
scatter(robotPoses(testingIndices,12), gpPredictedRobotPoses(:,12));
xlabel('Observed Z Position (m)');
ylabel('Desired Z Position (m)');

%% gaussian mixtures
n_comp = 10;
use_pca = true;
plot_2d = false;
use_AIC = false;
n_replicates = 20;
max_iter = 400;
color_plot = true;
%testData = [gpDataError.rotationError.eulerError gpDataError.translationError.rawError];

% create pose / robot gaussian mixture model
n_comp_both = 15;
use_pca_both = false;
plot_2d_both = true;

bothPoses = [cameraPoses(trainingIndices,1:12) robotPoses(trainingIndices,1:12), cameraGradients(trainingIndices,1:12)];
bothGMM = buildGMM(bothPoses, n_comp_both, use_pca_both, ...
    plot_2d_both, use_AIC, n_replicates, max_iter);

%% save gmm
gmmSave = cell(1,4);
gmmSave{1} = bothGMM.NComponents;
gmmSave{2} = bothGMM.PComponents;
gmmSave{3} = bothGMM.mu;
gmmSave{4} = bothGMM.Sigma;
save('results/GMM/gmmModel.mat', 'gmmSave');

%% get max residuals within radius
T = rigidErrorModel.T;
R = 0.015;
N = 2;
alpha = 0.01;
nSamples = 1e6;
nDim = 3;

labels = cluster(bothGMM, bothPoses);
sysTxError = sysTestError.translationError.rawError(find(labels==1),:);
mu = mean(sysTxError);
Sigma = cov(sysTxError);

inTxIndices = [4 8 12];
outTxIndices = [16 20 24];
hyperIndices = [inTxIndices outTxIndices];

interval = 20;
replans = 10;
MTTF = zeros(interval, 1);
prob = zeros(interval, 1);
radii = 0.001*(1:interval) + 0.005;
i = 1;

for R = radii
    samples = randsphere(nSamples, nDim, R);
    % mu = bothGMM.mu(1, hyperIndices);
    % Sigma = bothGMM.Sigma(hyperIndices, hyperIndices, 1);
    %samples = samples + repmat(mu, nSamples, 1);

    Vol = R^3 * 4 * pi / 3;

    y = mvnpdf(samples, mu, Sigma);
    probSuccess = Vol / nSamples * sum(y);
    probSuccess =  min(probSuccess, 1.0);
    prob(i, 1) = probSuccess;
    
    meanTime = 1 - probSuccess;
    cumulative = 0;
    for t = 1:1000
        meanTime = meanTime + (t+1) * exp(-alpha*cumulative) * ...
            probSuccess^t * (1 - exp(-alpha*t)*probSuccess);
        cumulative = cumulative + t;
    end
    
    MTTF(i, 1) = meanTime;%1.0 / (1.0 - probSuccess);
    i = i+1;
end

MTTF_replans = zeros(interval, replans);
for N = 1:replans
    MTTF_replans(:,N) = MTTF(:,1).^N;
end
%%
figure;
plot(radii, MTTF);
xlabel('Radius (m)');
ylabel('MTTF (attempts)');
title(sprintf('Mean time to failure vs Radius, alpha = %f', alpha));

%% plot GMM versus trajectory
figure;
translationIndices = [4 8 12];
testPoses = [cameraPoses(testingIndices,1:12) robotPoses(testingIndices,1:12), cameraGradients(testingIndices,1:12)];
testPoses = bothPoses;
labels = cluster(bothGMM, testPoses);
colors = ['y', 'm', 'c', 'r', 'g', 'b', 'w', 'k'];
for i = 1:bothGMM.NComponents
    color = colors(mod(i,size(colors,2))+1);
    indices = find(labels==i);
    scatter3(testPoses(indices,4), testPoses(indices,8), testPoses(indices,12), 80, ...
            'MarkerEdgeColor', 'k', 'MarkerFaceColor', color);

    hold on;
    scatter3(bothGMM.mu(i,4), bothGMM.mu(i,8), bothGMM.mu(i,12), 200.0, ...
           'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');

end

%% calculate the max residuals in the translation directions

T = rigidErrorModel.T;
gamma = 7.815;
eps = 1e-4;
n_dim = 4;%size(bothPoses,2);
n_trans = 3;

rotationIndices1 = [1 2 3 5 6 7 9 10 11];
rotationIndices2 = [13 14 15 17 18 19 21 22 23];
inTxIndices = [4 8 12];
outTxIndices = [16 20 24];

maxResiduals = cell(1, n_trans);
maxResidualPoses = cell(1, n_trans);
bias = T(1:3,4);

fprintf('\nCalculating max residuals...\n');
for i = 1:n_trans
    fprintf(sprintf('Checking translation component %d...\n',i));
    hyperIndices = [inTxIndices outTxIndices(i)];
    maxResiduals{i} = zeros(bothGMM.NComponents, 1);
    maxResidualPoses{i} = zeros(bothGMM.NComponents, n_dim);

    for clust = 1:bothGMM.NComponents
        mu_tx = bothGMM.mu(clust, hyperIndices)';
        Sigma_tx = bothGMM.Sigma(hyperIndices, hyperIndices, clust);

        % solve for the max residual
        r = T(i,1:3);
        w = [r -1]';%[zeros(1,3) r(1) zeros(1,3) r(2) zeros(1,3) r(3) zeros(1,3) -1 zeros(1,8)]';
        L = chol(inv(Sigma_tx));

        % solve the QPs
        cvx_begin quiet
            variable x(n_dim)
            variable z(n_dim)
            maximize(w' * x)
            subject to
                z == L * (x - mu_tx);
                z' * z <= gamma;
        cvx_end
        maxResidual = abs(cvx_optval + bias(i)) / norm(w);
        maxResidualPoses{i}(clust,:) = x';

        cvx_begin quiet
            variable y(n_dim)
            variable z(n_dim)
            maximize(-w' * y)
            subject to
                z == L * (y - mu_tx);
                z' * z <= gamma;
        cvx_end
        if maxResidual < abs(cvx_optval + bias(i)) / norm(w)   
            maxResidual = abs(cvx_optval + bias(i)) / norm(w);
            maxResidualPoses{i}(clust,:) = y';
        end
        maxResiduals{i}(clust) = maxResidual;

        fprintf(sprintf('Max residual for component %d: %f\n', clust, maxResidual));
    end
end

%% print out the translation norm error
maxResidualNorms = zeros(bothGMM.NComponents, 1);
for i = 1:bothGMM.NComponents
    clustRes = [maxResiduals{1}(i) maxResiduals{2}(i) maxResiduals{3}(i)];
    clustRes = norm(clustRes);
    maxResidualNorms(i) = clustRes;
    fprintf(sprintf('Norm residual for component %d: %f\n', i, clustRes));  
end

%% look at correlation between camera gradients and residuals
errorPoseVectors = [gpTestError.rotationError.eulerError ...
    gpTestError.translationError.rawError];

n_poses = size(rigidErrorModel.testTrueRobotPoses,1);
rVectors = zeros(n_poses, 6);
cVectors = zeros(n_poses, 6);
gVectors = zeros(n_poses, 6);
for i = 1:n_poses
    rVectors(i,:) = poseMatrix2Vector(rigidErrorModel.testTrueRobotPoses(i,:));
    cVectors(i,:) = poseMatrix2Vector(rigidErrorModel.testPredictedRobotPoses(i,:));
    gVectors(i,:) = cVectors(i,:) - rVectors(i,:);
end

errorNorms = sqrt(sum(abs(errorPoseVectors).^2,2));
gradNorms = sqrt(sum(abs(cameraGradientVectors).^2,2));

errorIndices = 1:6;
rPoseIndices = 7:12;
cPoseIndices = 13:18;
cGradIndices = 19:25;

X = [errorPoseVectors robotPoseVectors cameraPoseVectors, ...
    cameraGradientVectors, gradNorms];
[R, P] = corrcoef(X);
P_robot = P(errorIndices, rPoseIndices);
P_cam = P(errorIndices, cPoseIndices);
P_grad = P(errorIndices, cGradIndices);

S_robot = P_robot > 0.05;
S_cam = P_cam > 0.05;
S_grad = P_grad > 0.05;

correlations = struct();
correlations.S_robot = S_robot;
correlations.S_cam = S_cam;
correlations.S_grad = S_grad;
correlations.P_robot = P_robot;
correlations.P_cam = P_cam;
correlations.P_grad = P_grad;

%% max per-pose residual from the gaussian processes
index = 10000;
translationIndices = [4 8 12];

Sigma = diag(gpModel.testing_y_var(index, translationIndices));
mu = gpPredictedRobotPoses(index, translationIndices)';
[U S V] = svd(inv(Sigma));
lambda = eig(inv(Sigma));

L = chol(inv(Sigma));
gamma = 7.815;

cvx_begin
    variable alpha(3)
    maximize(sum(alpha))
    subject to
        alpha >= 0;
        lambda'*alpha <= gamma;
cvx_end

innerProd = sqrt(alpha);
res = U' * innerProd
%%
labels = cluster(myGMM, gpTestError.translationError.rawError);
[d,p] = manova1(gpTestError.translationError.rawError, labels);

clust1 = gpTestError.translationError.rawError(find(labels == 1),:);
clust2 = gpTestError.translationError.rawError(find(labels == 2),:);
p1 = signrank(clust1);
p2 = signrank(clust2);
%pAll = signrank(gpTestError.translationError.rawError);

%%
figure(10);
indices = [4 8 12];
translations = cameraPoses(:,indices);
scatter3(translations(:,1), translations(:,2), translations(:,3));
hold on;

mu = bothGMM.mu(:,indices);
scatter3(mu(:,1), mu(:,2), mu(:,3), 120.0, ...
         'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');

zlabel('Z');
xlabel('X');
ylabel('Y');
