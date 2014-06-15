
dataSize = 1; % for left arm
x = [];
y = [];
z = [];
euler_x = [];
euler_y = [];
euler_z = [];
std_x = [];
std_y = [];
std_z = [];
std_euler_x = [];
std_euler_y = [];
std_euler_z = [];
times = [];
trainingPerc = [];
trainingSize = [];
numTraj = [];
diffs = [];
directory = 'results/trainingSize/random';
testDataFiles = dir(directory);
index = 1;

for i = 1:size(testDataFiles,1)
    filename = testDataFiles(i).name;
    if size(filename,2) > 3 & filename(size(filename,2)-3:size(filename,2)) == '.mat'
        load(sprintf('%s/%s', directory, filename));
        x(index) = testErrorSave{4}.translationError.avgError(1);
        y(index) = testErrorSave{4}.translationError.avgError(2);
        z(index) = testErrorSave{4}.translationError.avgError(3);
        euler_x(index) = rad2deg(testErrorSave{4}.rotationError.meanEulerError(1));
        euler_y(index) = rad2deg(testErrorSave{4}.rotationError.meanEulerError(2));
        euler_z(index) = rad2deg(testErrorSave{4}.rotationError.meanEulerError(3));
        std_x(index) = testErrorSave{4}.translationError.stdError(1);
        std_y(index) = testErrorSave{4}.translationError.stdError(2);
        std_z(index) = testErrorSave{4}.translationError.stdError(3);
        std_euler_x(index) = rad2deg(testErrorSave{4}.rotationError.stdEulerError(1));
        std_euler_y(index) = rad2deg(testErrorSave{4}.rotationError.stdEulerError(2));
        std_euler_z(index) = rad2deg(testErrorSave{4}.rotationError.stdEulerError(3));
        
        times(index) = testErrorSave{5};
        trainingPerc(index) = testErrorSave{1};
        trainingSize(index) = testErrorSave{6};
        numTraj(index) = testErrorSave{7};
        %diffs(index) = testErrorSave{2};
        index = index+1;
    end
end
%% plot error vs training size
figure();
subplot(3,2,1);
plot(dataSize*trainingSize, x, '-ob', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'LineWidth', 2);
%title('X position Mean Error');
xlabel('Number of States in Training Set', 'FontSize', 10);
ylabel('X Mean Error (m)', 'FontSize', 10);

subplot(3,2,2);
plot(dataSize*trainingSize, y, '-ob', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'LineWidth', 2);
%title('z euler mean Error');
xlabel('Number of States in Training Set', 'FontSize', 10);
ylabel('Y Mean Error (m)', 'FontSize', 10);

subplot(3,2,3);
plot(dataSize*trainingSize, z, '-ob', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'LineWidth', 2);
%title('z euler mean Error');
xlabel('Number of States in Training Set', 'FontSize', 10);
ylabel('Z Mean Error (m)', 'FontSize', 10);

subplot(3,2,4);
plot(dataSize*trainingSize, euler_x, '-ob', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'LineWidth', 2);
%title('z euler mean Error');
xlabel('Number of States in Training Set', 'FontSize', 10);
ylabel('Yaw Mean Error (deg)', 'FontSize', 10);

subplot(3,2,5);
plot(dataSize*trainingSize, euler_y, '-ob', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'LineWidth', 2);
%title('z euler mean Error');
xlabel('Number of States in Training Set', 'FontSize', 10);
ylabel('Pitch Mean Error (deg)', 'FontSize', 10);

subplot(3,2,6);
plot(dataSize*trainingSize, euler_z, '-ob', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'LineWidth', 2);
%title('z euler mean Error');
xlabel('Number of States in Training Set', 'FontSize', 10);
ylabel('Roll Mean Error (deg)', 'FontSize', 10);

%% alternative?
figure();
subplot(2,1,1);
plot(trainingSize(1:7), x(1:7), '-or', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'LineWidth', 2);
hold on;
plot(trainingSize(1:7), y(1:7), '-^g', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'LineWidth', 2);
plot(trainingSize(1:7), z(1:7), '-sb', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'LineWidth', 2);
%title('z euler mean Error');
xlabel('Number of States in Training Set', 'FontSize', 15);
ylabel('Mean Error (m)', 'FontSize', 15);
legend('X', 'Y', 'Z', 'Location', 'Best');


subplot(2,1,2);
plot(trainingSize(1:7), euler_x(1:7), '-or', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'LineWidth', 2);

hold on;
plot(trainingSize(1:7), euler_y(1:7), '-^g', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'LineWidth', 2);
plot(trainingSize(1:7), euler_z(1:7), '-sb', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'LineWidth', 2);
%title('z euler mean Error');
xlabel('Number of States in Training Set', 'FontSize', 15);
ylabel('Mean Error (deg)', 'FontSize', 15);
legend('Yaw', 'Pitch', 'Roll', 'Location', 'Best');

%% alternative w diff
figure();
subplot(2,1,1);
errorbar(diffs, x, std_x, '-or', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'LineWidth', 2);
hold on;
errorbar(diffs, y, std_y, '-^g', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'LineWidth', 2);
errorbar(diffs, z, std_z, '-sb', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'LineWidth', 2);
%title('z euler mean Error');
xlabel('Gradient difference', 'FontSize', 15);
ylabel('Mean Error (m)', 'FontSize', 15);
legend('X', 'Y', 'Z', 'Location', 'Best');


subplot(2,1,2);
errorbar(diffs, euler_x, std_euler_x, '-or', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'LineWidth', 2);

hold on;
errorbar(diffs, euler_y, std_euler_y, '-^g', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'LineWidth', 2);
errorbar(diffs, euler_z, std_euler_z, '-sb', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'LineWidth', 2);
%title('z euler mean Error');
xlabel('Gradient difference', 'FontSize', 15);
ylabel('Mean Error (deg)', 'FontSize', 15);
legend('Yaw', 'Pitch', 'Roll', 'Location', 'Best');

%% alternative w traj
figure();
subplot(2,1,1);
plot(numTraj, x, '-or', 'MarkerFaceColor', 'r', 'MarkerSize', 5, 'LineWidth', 2);
hold on;
plot(numTraj, y, '-og', 'MarkerFaceColor', 'g', 'MarkerSize', 5, 'LineWidth', 2);
plot(numTraj, z, '-ob', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'LineWidth', 2);
%title('z euler mean Error');
xlabel('Number of Trajectories in Training Set', 'FontSize', 15);
ylabel('Mean Error (m)', 'FontSize', 15);
legend('X', 'Y', 'Z', 'Location', 'Best');


subplot(2,1,2);
plot(numTraj, euler_x, '-or', 'MarkerFaceColor', 'r', 'MarkerSize', 5, 'LineWidth', 2);

hold on;
plot(numTraj, euler_y, '-og', 'MarkerFaceColor', 'g', 'MarkerSize', 5, 'LineWidth', 2);
plot(numTraj, euler_z, '-ob', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'LineWidth', 2);
%title('z euler mean Error');
xlabel('Number of Trajectories in Training Set', 'FontSize', 15);
ylabel('Mean Error (deg)', 'FontSize', 15);
legend('Yaw', 'Pitch', 'Roll', 'Location', 'Best');


%% plot time vs training size
figure;
plot((dataSize*trainingSize), times, '-ob', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'LineWidth', 2);
%title('Training time versus data size');
xlabel('Number of States in Training Set', 'FontSize', 15);
ylabel('Training Time (s)', 'FontSize', 15);

%% plot time vs training size in traj
figure;
plot(numTraj, times, '-ob', 'MarkerFaceColor', 'b', 'MarkerSize', 5, 'LineWidth', 2);
%title('Training time versus data size');
xlabel('Number of Trajectories in Training Set', 'FontSize', 15);
ylabel('Training Time (s)', 'FontSize', 15);