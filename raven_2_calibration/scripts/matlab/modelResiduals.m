function [ output_args ] = modelResiduals(T, cameraPoses, robotPoses)
  meanfunc = {@meanSum, {@meanLinear, @meanConst}};
  covfunc = {@covMaterniso, 3};
  hyp.mean = [0.5; 1];
  ell = 1/4; sf = 1;
  hyp.cov = log([ell; sf]);
  likfunc = @likGauss;
  sn = 0.1;
  hyp.lik = log(sn);

  
  K = feval(covfunc{:}, hyp.cov, x);
  mu = feval(meanfunc{:}, hyp.mean, x);
  y = chol(K)'*gpml_randn(0.15, n, 1) + mu + exp(hyp.lik)*gpml_randn(0.2, n, 1);

  plot(x, y, '+')
end

