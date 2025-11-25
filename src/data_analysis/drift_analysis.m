%% Load CSV
data = readtable('odometry2.csv');   % columns: ticks, angle

ticks  = data.ticks;
angles = data.angle;

N = length(ticks);

%% --- Step 1: Convert ticks+angle to raw XY coordinates ---
x = zeros(N,1);
y = zeros(N,1);

for i = 2:N
    dist = ticks(i) - ticks(i-1);
    theta = angles(i);

    dx = dist * cosd(theta);
    dy = dist * sind(theta);

    x(i) = x(i-1) + dx;
    y(i) = y(i-1) + dy;
end


%% --- Step 2: Set first 100 samples as reference drift-free region ---
window = 100;

ref_x = x(1:window);
ref_y = y(1:window);

ref_mean_x = mean(ref_x);
ref_mean_y = mean(ref_y);

%% --- Step 3: Drift correction for each 100-point block ---
x_corrected = x;
y_corrected = y;

num_blocks = floor(N / window);

for b = 2:num_blocks   % block 1 is the reference
    idx_start = (b-1)*window + 1;
    idx_end   = b*window;

    block_x = x(idx_start:idx_end);
    block_y = y(idx_start:idx_end);

    % Drift = difference between block mean and reference mean
    drift_x = mean(block_x) - ref_mean_x;
    drift_y = mean(block_y) - ref_mean_y;

    % Remove drift
    x_corrected(idx_start:idx_end) = block_x - drift_x;
    y_corrected(idx_start:idx_end) = block_y - drift_y;
end


%% --- Step 4: Plot results ---
figure;
subplot(1,2,1);
plot(x, y, '-o', 'LineWidth', 1.5);
title('Raw XY Path (With Drift)');
xlabel('X');
ylabel('Y');
axis equal;
grid on;

subplot(1,2,2);
plot(x_corrected, y_corrected, '-o', 'LineWidth', 1.5);
title('Drift-Corrected XY Path');
xlabel('X');
ylabel('Y');
axis equal;
grid on;

sgtitle('Odometry Drift Analysis (100-sample correction)');