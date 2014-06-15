function [testError, gpModel, testPredictedRobotPoses, testTrueRobotPoses, tCameraPoses, allPredictedRobotPoses] = ...
    estimateResidualModel(trainingCameraPoses, trainingRobotPoses, trainingGradients,...
    testingCameraPoses, testingRobotPoses, testingGradients, n_iter)
    
    if nargin < 7
       n_iter = 25; 
    end

    % set up the training indices and testing indices
    M = size(testingCameraPoses, 1);
    N = size(trainingCameraPoses, 1);
    
    % run the regression
    n_dim = 12;
    n_testing = M;
    
    training_x = [trainingCameraPoses trainingGradients];
    testing_x = [testingCameraPoses testingGradients];
    
    training_y = trainingRobotPoses;
    testing_y = testingRobotPoses;
    testing_y_predicted = zeros(n_testing, n_dim);
    %all_x = trainingCameraPoses(1:N,:); %[trainingCameraPoses(1:N,:) trainingGradients(1:N,:)];
    all_y_predicted = zeros(N, n_dim);

    testing_y_variances = zeros(n_testing, n_dim);
    all_y_variances = zeros(N, n_dim);
    
    meanfunc = {@meanSum, {@meanLinear, @meanConst}};
    nMeanHyp = size(testing_x, 2) + 1;
    hyp.mean = ones(nMeanHyp,1);
    covfunc = {@covMaterniso, 3}; ell = 1/4; sf = 1;
    hyp.cov = log([ell; sf]);
    likfunc = @likGauss; sn = 0.1; hyp.lik = log(sn);
    
    gpModel = struct();
    gpModel.alpha = cell(1,n_dim);
    gpModel.meanFunc = meanfunc;
    gpModel.covFunc = covfunc;
    gpModel.likFunc = likfunc;
    gpModel.training_x = training_x;
    gpModel.training_y = training_y;
    gpModel.testing_x = testing_x;
    gpModel.testing_y = testing_y;
    gpModel.hyp = cell(1,n_dim);

    for i = 1:n_dim
        fprintf(sprintf('Predicting dimension %d\n', i));
        hyp.cov = [0; 0]; hyp.mean = zeros(nMeanHyp, 1); hyp.lik = log(0.1);
        hyp = minimize(hyp, @gp, -n_iter, @infExact, meanfunc, covfunc, likfunc, training_x, training_y(:,i));    
        
        % compute alpha from the data
        gpModel.hyp{i} = hyp;
        Kxx = feval(covfunc{:}, hyp.cov, training_x);
        Mx = feval(meanfunc{:}, hyp.mean, training_x);
        beta = exp(2*hyp.lik);                             
        if beta < 1e-6                 % very tiny sn2 can lead to numerical trouble
          L = chol(Kxx + beta*eye(N)); % Cholesky factor of covariance with noise
          sl = 1;
        else
          L = chol(Kxx/beta + eye(N));     % Cholesky factor of B
          sl = beta; 
        end
        alpha = solve_chol(L, training_y(:,i) - Mx) / sl;
        gpModel.alpha{i} = alpha;
        
        [testing_y_predicted(:,i), testing_y_variances(:,i)] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, training_x, training_y(:,i), testing_x);
 %       [all_y_predicted(:,i), all_y_variances(:,i)] = gp(hyp, @infExact, meanfunc, covfunc, likfunc, training_x, training_y(:,i), all_x);
    end
    
    gpModel.testing_y_var = testing_y_variances;
    gpModel.all_y_var = all_y_variances;
    
    testPredictedRobotPoses = [testing_y_predicted zeros(n_testing,3) ones(n_testing,1)];
    testTrueRobotPoses = testing_y;
    tCameraPoses = testing_x;
    allPredictedRobotPoses = [all_y_predicted zeros(N,3) ones(N,1); zeros(1,16)];
    
    testError = struct();
    [testError.matrixError, testError.translationError, testError.rotationError] = ...
        evaluateErrors(testPredictedRobotPoses, testTrueRobotPoses);
      