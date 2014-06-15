function [residuals] = boundResiduals(T, cameraPoses, n_comp)

    if nargin < 3
        n_comp = 10;
    end

    AIC = zeros(1,n_comp);
    BIC = zeros(1,n_comp);
    obj = cell(1,n_comp);

    n_replicates = 40;
    options = statset('MaxIter', 500, 'Display', 'off', 'TolFun', 1e-6); 

    for k = 1:n_comp
        obj{k} = gmdistribution.fit(cameraPoses(:,1:12), k, 'CovType','full', ...
             'Replicates', n_replicates, 'Options', options));
        AIC(k)= obj{k}.AIC;
        BIC(k)= obj{k}.BIC;
    end

    [minAIC, numComponentsA] = min(AIC);
    [minBIC, numComponentsB] = min(BIC);
    bestGMM = obj{numComponentsB};

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
end

