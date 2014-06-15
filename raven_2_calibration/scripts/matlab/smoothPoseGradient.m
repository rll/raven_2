function g = smoothPoseGradient(p_cur, p_next, p_prev, n_dt, p_dt)
%                     [dy, dp, dr] = dcm2angle(d(1:3,1:3));
%                     dy = dy / c_dt;
%                     dp = dp / c_dt;
%                     dr = dr / c_dt;
% d(1:3,1:3) = angle2dcm(dy, dp, dr);

    g_prev = inv(p_prev) * p_cur;
    g_next = inv(p_cur) * p_next;
    g_smooth = inv(p_prev) * p_next;
    
    g_prev(1:3,4) = g_prev(1:3,4) / p_dt;
    g_next(1:3,4) = g_next(1:3,4) / n_dt;
    g_smooth(1:3,4) = g_smooth(1:3,4) / (p_dt + n_dt);
    g = 0.4 * g_next + 0.4 * g_prev + 0.2 * g_smooth;
end

