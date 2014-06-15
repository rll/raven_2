function [cameraPoses, robotPoses, commandPoses, ...
    cameraGradients, robotGradients, commandGradients, ...
    targetPoses, trajStartIndices, trajEndIndices] = ...
    loadTrajectories(directories, subsampleRate, commandLatency, clean, diff, standardize)

cameraPoses = zeros(0,16);
robotPoses = zeros(0,16);
commandPoses = zeros(0,16);
targetPoses = zeros(0,16);

trajStartIndices = [];
trajEndIndices = [];

cameraGradients = zeros(0, 16);
robotGradients = zeros(0, 16);
commandGradients = zeros(0, 16);

D = subsampleRate; % subsampling rate
curI = 1;
curJ = 1;    
    
for a = 1:size(directories,2)
    fprintf(sprintf('Loading directory %s at trajectory index %d\n', directories{a}, curJ));
    trajectoryFilesRaw = dir(directories{a});
    ix = randperm(size(trajectoryFilesRaw,1));
    trajectoryFiles = trajectoryFilesRaw(ix);

    c = [];
    r = [];
    z = [];
    cs = [];
    rs = [];
    zs = [];

    %for i = trajStartI:trajEndI
    for i = 1:size(trajectoryFiles,1)
        %filename = sprintf(filenameTemplate, i, arm);
        filename = trajectoryFiles(i).name;
        if size(filename,2) > 3 & filename(size(filename,2)-3:size(filename,2)) == '.mat'
            load(filename);

            M = size(c,1);

            c = c(commandLatency:M,:);
            cs = cs(commandLatency:M,:);
            r = r(commandLatency:M,:); 
            rs = rs(commandLatency:M,:);
            z = z(1:(M-commandLatency+1),:); 
            zs = zs(1:(M-commandLatency+1),:); 
            
            M = size(c,1);
            
            c_g = zeros(size(c));
            r_g = zeros(size(r));
            z_g = zeros(size(z));
            index = 1;
            validIndices = [];
            
            for j = (2*diff+1):(M-2*diff)
                c_cur = reshape(c(j,:), 4, 4)';
                r_cur = reshape(r(j,:), 4, 4)';
                z_cur = reshape(z(j,:), 4, 4)';
                
                c_prev = reshape(c(j-diff,:), 4, 4)';
                r_prev = reshape(r(j-diff,:), 4, 4)';
                z_prev = reshape(z(j-diff,:), 4, 4)';
                
                c_next = reshape(c(j+diff,:), 4, 4)';
                r_next = reshape(r(j+diff,:), 4, 4)';
                z_next = reshape(z(j+diff,:), 4, 4)';
                
                c_next2 = reshape(c(j+2*diff,:), 4, 4)';
                r_next2 = reshape(r(j+2*diff,:), 4, 4)';
                z_next2 = reshape(z(j+2*diff,:), 4, 4)';
                
                c_prev2 = reshape(c(j-2*diff,:), 4, 4)';
                r_prev2 = reshape(r(j-2*diff,:), 4, 4)';
                z_prev2 = reshape(z(j-2*diff,:), 4, 4)';
                
                c_dt = cs(j,:) - cs(j-diff,:);
                r_dt = rs(j,:) - rs(j-diff,:);
                z_dt = zs(j,:) - zs(j-diff,:);
                
                c_ndt = cs(j+diff,:) - cs(j,:);
                r_ndt = rs(j+diff,:) - rs(j,:);
                z_ndt = zs(j+diff,:) - zs(j,:);
                
                c_ndt2 = cs(j+2*diff,:) - cs(j+diff,:);
                r_ndt2 = rs(j+2*diff,:) - rs(j+diff,:);
                z_ndt2 = zs(j+2*diff,:) - zs(j+diff,:);
                
                c_pdt2 = cs(j-diff,:) - cs(j-2*diff,:);
                r_pdt2 = rs(j-diff,:) - rs(j-2*diff,:);
                z_pdt2 = zs(j-diff,:) - zs(j-2*diff,:);
   
                if diff == 0
                    c_dt = 1;
                    r_dt = 1;
                    z_dt = 1;
                    c_ndt = 1;
                    r_ndt = 1;
                    z_ndt = 1;
                    c_ndt2 = 1;
                    r_ndt2 = 1;
                    z_ndt2 = 1;
                    c_pdt2 = 1;
                    r_pdt2 = 1;
                    z_pdt2 = 1;
                end
                
                if c_dt > 0 && r_dt > 0 && z_dt > 0 && ...
                   c_ndt > 0 && r_ndt > 0 && z_ndt > 0 && ...
                   c_ndt2 > 0 && r_ndt2 > 0 && z_ndt2 > 0 && ...
                   c_pdt2 > 0 && r_pdt2 > 0 && z_pdt2 > 0
                   
                    %d = smoothPoseGradient(c_cur, c_next, c_prev, c_ndt, c_dt);
                    d = smoothPoseGradient5Point(c_cur, c_next, c_prev, c_next2, c_prev2, c_ndt, c_dt, c_ndt2, c_pdt2);
                    %d = poseGradient(c_cur, c_prev, c_dt);
                    c_g(index,:) = reshape(d', 1, 16);

                    %d = smoothPoseGradient(r_cur, r_next, r_prev, r_ndt, r_dt);
                    d = smoothPoseGradient5Point(r_cur, r_next, r_prev, r_next2, r_prev2, r_ndt, r_dt, r_ndt2, r_pdt2);
                    %d = poseGradient(r_cur, r_prev, r_dt);
                    r_g(index,:) = reshape(d', 1, 16);

                    %d = smoothPoseGradient(z_cur, z_next, z_prev, z_ndt, z_dt);
                    d = smoothPoseGradient5Point(z_cur, z_next, z_prev, z_next2, z_prev2, z_ndt, z_dt, z_ndt2, z_pdt2);
                    %d = poseGradient(z_cur, z_prev, z_dt);
                    z_g(index,:) = reshape(d', 1, 16);

                    index = index+1;
                    validIndices = [validIndices j];
                end
            end

            % remove indices with 0 gradient
            c = c(validIndices, :);
            r = r(validIndices, :);
            z = z(validIndices, :);
            
            % determine sizes
            M = size(c, 1);
            N = floor(M / D);
            indices = (1:N)*D;
            indices = setdiff(indices, 1); % remove the 1 index since there is no gradient
            N = size(indices,2);
            
            % subsample the indices
            c_sampled = c(indices, :);
            r_sampled = r(indices, :);
            z_sampled = z(indices, :);
            c_g_sampled = c_g(indices, :);
            r_g_sampled = r_g(indices, :);
            z_g_sampled = z_g(indices, :);

            reject = false;
            if clean
                [c_sampled, r_sampled, z_sampled, c_g_sampled, r_g_sampled, z_g_sampled, numReject, reject] = ...
                    cleanTrajectories(c_sampled, r_sampled, z_sampled, c_g_sampled, r_g_sampled, z_g_sampled);
            end
            if ~reject
                N = size(c_sampled,1);

                trajStartIndices(curJ) = curI;
                trajEndIndices(curJ) = curI+N-1;

                cameraPoses(curI:(curI+N-1),:) = c_sampled;
                robotPoses(curI:(curI+N-1),:) = r_sampled;
                commandPoses(curI:(curI+N-1),:) = z_sampled;
                cameraGradients(curI:(curI+N-1),:) = c_g_sampled;
                robotGradients(curI:(curI+N-1),:) = r_g_sampled;
                commandGradients(curI:(curI+N-1),:) = z_g_sampled;
                targetPoses(curI:(curI+N-1),:) = repmat(reshape(reshape(t(1,:), 4, 4)', 1, 16), N, 1);

                curJ = curJ + 1;
                curI = curI + N;
            end
        end
    end
end

if standardize
    %cameraPoses = zscore(cameraPoses);
    %robotPoses = zscore(robotPoses);
    %commandPoses = zscore(commandPoses);
    cameraGradients = repmat(std(cameraPoses,1),size(cameraGradients,1),1).*zscore(cameraGradients) + repmat(mean(cameraPoses,1),size(cameraGradients,1),1);
    robotGradients = repmat(std(robotPoses,1),size(cameraGradients,1),1).*zscore(robotGradients) + repmat(mean(robotPoses,1),size(cameraGradients,1),1);
    commandGradients = repmat(std(commandPoses,1),size(cameraGradients,1),1).*zscore(commandGradients) + repmat(mean(commandPoses,1),size(cameraGradients,1),1);
end
