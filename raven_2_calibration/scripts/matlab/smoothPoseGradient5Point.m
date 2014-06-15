function g = smoothPoseGradient5Point(p_cur, p_next, p_prev,  p_next2, p_prev2, n_dt, p_dt, n2_dt, p2_dt)
%                     [dy, dp, dr] = dcm2angle(d(1:3,1:3));
%                     dy = dy / c_dt;
%                     dp = dp / c_dt;
%                     dr = dr / c_dt;
% d(1:3,1:3) = angle2dcm(dy, dp, dr);

    g_next2 = p_next2 - p_next;
    g_next = p_next - p_cur;
    g_prev = p_cur - p_prev;
    g_prev2 = p_prev - p_prev2;
    
    total_time = n_dt + p_dt + n2_dt + p2_dt;
    n2_frac = n2_dt / total_time;
    n_frac = n_dt / total_time;
    p_frac = p_dt / total_time;
    p2_frac = p2_dt / total_time;
    
    g = -1 * n2_frac * g_next2 + 7 * n_frac * g_next - ...
        7 * p_frac * g_prev + 1 * p2_frac * g_prev2;
    g = g / 3;
end

