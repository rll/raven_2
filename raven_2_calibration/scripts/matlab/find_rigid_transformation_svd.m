function [R, t] = find_rigid_transformation_svd(r, c)
    [N ~] = size(r);

    bar_p = zeros(3, 1);
    bar_q = zeros(3, 1);
    for i=1:N
        T_c = reshape(c(i,:),4,4)';
        T_r = reshape(r(i,:),4,4)';
        bar_p = bar_p + T_c(1:3,4) / N;
        bar_q = bar_q + T_r(1:3,4) / N;
    end

    X = zeros(3,N);
    Y = zeros(3,N);
    for i=1:N
        T_c = reshape(c(i,:),4,4)';
        T_r = reshape(r(i,:),4,4)';
        x = T_c(1:3,4) - bar_p;
        y = T_r(1:3,4) - bar_q;
        X(:,i) = x;
        Y(:,i) = y;
    end

    S = X*Y';
    [U Sigma V] = svd(S);

    scale = det(V*U');
    scale_matrix = eye(3,3);
    scale_matrix(3,3) = scale;

    R = V*scale_matrix*U';
    t = bar_q - R*bar_p;
