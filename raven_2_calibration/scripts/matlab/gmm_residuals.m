
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
tau = T(1:3,4);

meanState = mean(bothPoses);
covState = cov(bothPoses);

fprintf('\nCalculating max residuals...\n');
for i = 1:n_trans
    fprintf(sprintf('Checking translation component %d...\n',i));
    hyperIndices = [inTxIndices outTxIndices(i)];
    maxResiduals{i} = zeros(bothGMM.NComponents,1);
    maxResidualPoses{i} = zeros(bothGMM.NComponents, n_dim);

    for clust = 1:shitty bothGMM.NComponents
        mu_tx = bothGMM.mu(clust, hyperIndices)';
        Sigma_tx = bothGMM.Sigma(hyperIndices, hyperIndices, clust);

%         mu_tx = meanState(hyperIndices)';
%         Sigma_tx = covState(hyperIndices, hyperIndices);
        
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
        maxResidual = (abs(cvx_optval + tau(i))) / norm(w);
        maxResidualPoses{i}(clust,:) = x';

        cvx_begin quiet
            variable y(n_dim)
            variable z(n_dim)
            maximize(-w' * y)
            subject to
                z == L * (y - mu_tx);
                z' * z <= gamma;
        cvx_end
        newResidual = (abs(cvx_optval + tau(i))) / norm(w);
        if maxResidual < newResidual  
            maxResidual = newResidual;
            maxResidualPoses{i}(clust,:) = y';
        end
        maxResiduals{i}(clust) = maxResidual;

        fprintf(sprintf('Max residual for component %d: %f\n', clust, maxResidual));
    end
end
%%
labels = cluster(bothGMM, bothPoses);
[D, P] = manova1(bothPoses(:,[inTxIndices outTxIndices(3)]), labels);

%%
inTxIndices = [4 8 12];
outTxIndices = [16 20 24];
poses = [cameraPoses(:,1:12) robotPoses(:,1:12)];
meanPose = mean(poses);
covPose = cov(poses);

%%
meanTxError = mean(sysDataError.translationError.rawError);
covTxError = cov(sysDataError.translationError.rawError);
worstTxError = abs(meanTxError) + sqrt(7.815)*sqrt(diag(covTxError)');
worstTxNorm = norm(worstTxError);

find(maxResidualNorms < worstTxNorm)

%%

[U S V] = svd(inv(Sigma));
lambda = eig(inv(Sigma));
L = chol(inv(Sigma));
gamma = 7.815;

% 
cvx_begin
    variable alpha(3)
    maximize(sum(alpha))
    subject to
        alpha >= 0;
        diag(S)'*alpha <= gamma;
cvx_end

innerProd = sqrt(alpha);
res = V * innerProd;
(res)' * L' * L * (res);

%%
% create residual gaussian mixture model
myGMM = buildGMM(testData, n_comp, use_pca, ...
    plot_2d, use_AIC, n_replicates, max_iter, color_plot);


%% look at correlation between camera gradients and residuals
rawErrorPoseVectors = [gpDataError.rotationError.eulerError gpDataError.translationError.rawError];
errorNorms = sqrt(sum(abs(rawErrorPoseVectors).^2,2));
gradNorms = sqrt(sum(abs(robotGradientVectors).^2,2));

X = [rawErrorPoseVectors errorNorms robotPoseVectors gradNorms];
[R, P] = corrcoef(X);
Rinteresting = R(1:7,8:14);
Pinteresting = P(1:7,8:14);
Significant = Pinteresting < 0.05;

%% now label camera poses / gradients based on gmm
gpResidualLabels = cluster(myGMM, testData);
colors = ['y', 'm', 'c', 'r', 'g', 'b', 'w', 'k'];
%grad = cameraPoses(:,1:16);
N = size(cameraPoses,1);
grad = zeros(N, 16);
for i = 1:(N-1)
    grad(i,:) = cameraPoses(i+1,:) - cameraPoses(i,:);
end
%
figure;
indices = [4,8,12];
N = myGMM.NComponents;
means = zeros(N,3);
stdErrs = zeros(3,3,N);
for i = 1:N
    color = colors(mod(i,size(colors,2))+1);
    gradients = grad(find(gpResidualLabels == i),indices);
    means(i,:) = mean(gradients);
    stdErrs(:,:,i) = cov(gradients);
    scatter3(gradients(:,1),...
             gradients(:,2),...
             gradients(:,3),...
             50.0, 'MarkerEdgeColor', [0 0 0],...
             'MarkerFaceColor', color);
    hold on;
end
%%
    %             zeros(3) <= reshape(x(rotationIndices1), 3, 3) <= eye(3);
    %             zeros(3) <= reshape(x(rotationIndices2), 3, 3) <= eye(3);
    %             x(1:3)' * x(1:3) <= 1;
    %             x(5:7)' * x(5:7) <= 1;
    %             x(9:11)' * x(9:11) <= 1;
    %             x(13:15)' * x(13:15) <= 1;
    %             x(17:19)' * x(17:19) <= 1;
    %             x(21:23)' * x(21:23) <= 1;
    %%
        %     diff = maxResidualPoses(1,:)' - mu_tx;
    %     z = L * diff;
    %     Pcam = reshape(maxResidualPoses(1,1:12),4,3)';
    %     Prob = reshape(maxResidualPoses(1,13:24),4,3)';
    %     [U S V] = svd(Pcam(1:3,1:3));
    %     Pcam(1:3,1:3) = U * V';
    %     [U S V] = svd(Prob(1:3,1:3));
    %     Prob(1:3,1:3) = U * V';
    %     v = zeros(1,n_dim);
    %     v(1:12) = reshape(Pcam',1,12);
    %     v(13:24) = reshape(Prob',1,12);
    %     trueRes = abs(v * w) / norm(w);
    %     trueZ = L * (v' - mu_tx);
    %     trueQuad = trueZ' * trueZ;