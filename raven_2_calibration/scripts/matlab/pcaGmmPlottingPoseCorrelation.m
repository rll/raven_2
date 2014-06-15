%% pca
transErrorPerDim = sysTestError.translationError.rawError;
[U, mu, vars] = pca(transErrorPerDim');
[Yk, Xhat, avsq] = pcaApply(transErrorPerDim', U, mu, 3);
figure;
%scatter(Yk(1,:)', Yk(2,:)');
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
bestGMM = obj{numComponentsB};

figure;
title('Systematic Residual Errors On First Two Principle Components (blue) with GMM Cluster Means(red)');
xlabel('First Principal Component');
ylabel('Second Principal Component');

% figure out what the poses were that generated thiss
idx = cluster(bestGMM, Yk');
colors = ['y', 'm', 'c', 'r', 'g', 'b', 'w', 'k'];

for i = 1:numComponentsB
    color = colors(mod(i,size(colors,2))+1);
    clusterIdx = find(idx == i);
   
    scatter3(bestGMM.mu(i,1), bestGMM.mu(i,2), bestGMM.mu(i,3), 150.0, 'MarkerEdgeColor', color,...
        'MarkerFaceColor', color);
    hold on;
    scatter3(Yk(1,clusterIdx)', Yk(2,clusterIdx)', Yk(3,clusterIdx)', 'MarkerEdgeColor', color);
end

avgTranslation = zeros(numComponentsB, 3);
cameraPoseTranslations = zeros(size(sysCameraPoses,1), 3);
cameraPoseTranslations(:,1) = sysCameraPoses(:,4);
cameraPoseTranslations(:,2) = sysCameraPoses(:,8);
cameraPoseTranslations(:,3) = sysCameraPoses(:,12);

figure;

for i = 1:numComponentsB
    color = colors(mod(i,size(colors,2))+1);
    tMat = cameraPoseTranslations(find(idx==i), :);
    avgTranslation(i,:) = mean(tMat, 1);
    scatter3(cameraPoseTranslations(find(idx==i),1), cameraPoseTranslations(find(idx==i),2), cameraPoseTranslations(find(idx==i),3));
    hold on;
    scatter3(avgTranslation(i,1), avgTranslation(i,2), ...
        avgTranslation(i,3), 150.0, 'MarkerEdgeColor', color,...
        'MarkerFaceColor', color);
end