function v = poseMatrix2Vector(M)
    T = reshape(M, 4, 4)';
    R = T(1:3,1:3);
    [yaw, pitch, roll] = dcm2angle(R);
    trans = T(1:3,4);
    v = [yaw pitch roll trans'];
end

