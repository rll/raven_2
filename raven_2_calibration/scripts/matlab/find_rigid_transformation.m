function [R, t] = find_rigid_transformation(r, c)
    [N ~] = size(r);

    bar_p = zeros(3,1);
    bar_q = zeros(3,1);
    avg_t_p = zeros(3,1);
    avg_t_q = zeros(3,1);
    for i = 1:N
        T_c = reshape(c(i,:),4,4)';
        T_r = reshape(r(i,:),4,4)';
        bar_p = bar_p + (T_c(1:3,1) + T_c(1:3,2) + T_c(1:3,3) + T_c(1:3,4)) / (4*N);
        bar_q = bar_q + (T_r(1:3,1) + T_r(1:3,2) + T_r(1:3,3) + T_r(1:3,4)) / (4*N);
        avg_t_p = avg_t_p + T_c(1:3,4) / N;
        avg_t_q = avg_t_q + T_r(1:3,4) / N;
    end

    X = zeros(3,4*N);
    Y = zeros(3,4*N);
    for i = 1:N
        T_c = reshape(c(i,:),4,4)';
        T_r = reshape(r(i,:),4,4)';
        x = T_c(1:3,:) - repmat(bar_p, 1, 4);
        y = T_r(1:3,:) - repmat(bar_q, 1, 4);
        X(:,i*4-3:i*4) = x;
        Y(:,i*4-3:i*4) = y;
    end

    S = X*Y';
    [U Sigma V] = svd(S);

    scale = det(V*U');
    scale_matrix = eye(size(scale));

    R = V*scale_matrix*U';
    t = avg_t_q - R*avg_t_p;






