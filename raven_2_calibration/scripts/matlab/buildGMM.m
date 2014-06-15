function [bestGMM] = buildGMM(data, n_components, use_pca, plot_2d, ...
use_AIC, n_replicates, max_iter, color_plot)
    
    if nargin < 2
        n_components = 10;
    end
    if nargin < 3
        use_pca = false;
    end
    if nargin < 4
        plot_2d = true;
    end
    if nargin < 5
        use_AIC = true;
    end
    if nargin < 6
        n_replicates = 40;
    end
    if nargin < 7
        max_iter = 500;
    end
    if nargin < 8
        color_plot = false;
    end

    options = statset('MaxIter', max_iter, 'Display', 'off', 'TolFun', 1e-6); 
    covType = 'full';

    AIC = zeros(1, n_components);
    BIC = zeros(1, n_components);
    obj = cell(1, n_components);

    % run pca if spcified
    convertedData = data;
    if use_pca
        pca_dim = 6;
        [U, mu, vars] = pca(data');
        [Yk, Xhat, avsq] = pcaApply(data', U, mu, pca_dim);
        convertedData = Yk';
    end

    for k = 1:n_components
        fprintf(sprintf('Generating %d component model...\n', k));
        obj{k} = gmdistribution.fit(convertedData, k, 'CovType', covType, ...
                    'Replicates', n_replicates, 'Options', options);
        AIC(k)= obj{k}.AIC;
        BIC(k)= obj{k}.BIC;
    end
    fprintf(sprintf('Done generating models\n', k));
        
    [minAIC, numComponentsA] = min(AIC);
    [minBIC, numComponentsB] = min(BIC);
    if use_AIC        
        bestGMM = obj{numComponentsA};
    else
        bestGMM = obj{numComponentsB};
    end

    % plot residuals w/ clusters
    figure(10);

    % plot data
    if plot_2d
        scatter(convertedData(:,1), convertedData(:,2), 'MarkerEdgeColor', 'b');
    else
        scatter3(convertedData(:,1), convertedData(:,2), convertedData(:,3), ...
            'MarkerEdgeColor', 'b');
    end
    hold on;

    % plot gmm centers
    if plot_2d
        scatter(bestGMM.mu(:,1), bestGMM.mu(:,2), 120.0, ...
            'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
    else
        scatter3(bestGMM.mu(:,1), bestGMM.mu(:,2), bestGMM.mu(:,3), 120.0, ...
            'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
        zlabel('Z');
    end
    xlabel('X');
    ylabel('Y');

    figure(11);
    plot([AIC' BIC']);
    title('AIC / BIC Versus Num Clusters');

    if color_plot
        nc = bestGMM.NComponents;
        IDX = cluster(bestGMM, convertedData);
        figure(12);
        colors = ['y', 'm', 'c', 'r', 'g', 'b', 'w', 'k'];
        for i = 1:nc
            color = colors(mod(i,size(colors,2))+1);
            if plot_2d
                scatter(convertedData(find(IDX == i),1),...
                    convertedData(find(IDX == i),2), ...
                    50.0, 'MarkerEdgeColor', [0 0 0],...
                    'MarkerFaceColor', color);
            else
                scatter3(convertedData(find(IDX == i),1),...
                    convertedData(find(IDX == i),2),...
                    convertedData(find(IDX == i),3),...
                    50.0, 'MarkerEdgeColor', [0 0 0],...
                    'MarkerFaceColor', color);
            end
            hold on;
        end
    end
end

