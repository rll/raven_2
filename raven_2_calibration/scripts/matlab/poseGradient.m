function g = poseGradient(p_cur, p_prev, dt)
%                     [dy, dp, dr] = dcm2angle(d(1:3,1:3));
%                     dy = dy / c_dt;
%                     dp = dp / c_dt;
%                     dr = dr / c_dt;
% d(1:3,1:3) = angle2dcm(dy, dp, dr);

    g = inv(p_prev) * p_cur;
    %q = (p_cur - p_prev) / dt;
    %g(1:3,1:3) = q(1:3,1:3) * p_cur(1:3,1:3)';
    g(1:3,4) = g(1:3,4) / dt;
end

