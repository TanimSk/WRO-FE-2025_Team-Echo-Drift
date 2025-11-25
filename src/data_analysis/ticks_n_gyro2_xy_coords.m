% Load CSV
data = readtable('odometry2.csv');   % Ensure columns: ticks, angle

data.Properties.VariableNames

ticks = data.ticks;
angles = data.angle;


% Initialize coordinate arrays
x = zeros(length(ticks), 1);
y = zeros(length(ticks), 1);

% Convert ticks to XY
for i = 2:length(ticks)
    dist = ticks(i) - ticks(i-1);       % distance moved
    theta = angles(i);                 % angle in degrees
    
    % Convert polar movement to Cartesian
    dx = dist * cosd(theta);
    dy = dist * sind(theta);
    
    % Update XY
    x(i) = x(i-1) + dx;
    y(i) = y(i-1) + dy;
end

% Plot result
figure;
plot(x, y, '-o', 'LineWidth', 2);
grid on;
axis equal;
xlabel('X Position');
ylabel('Y Position');
title('Path from Ticks + Angle');