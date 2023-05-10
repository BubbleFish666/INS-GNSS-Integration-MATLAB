%% set waypoints
clear; close all;
     % Groundspeed, Waypoints, Orientation
constraints = [0,     23,36,0,    -50,0,0;
               1.5,   38,46,0,    -45,0,0;
               1.48,  46,36,0,    -160,0,0;
               1.40,  52,7,0      -110,0,0;
               1.54,  78,18,0     -5,0,0;
               1.45,  64,39,0     10,0,0;
               1.47,  67,54,0     -20,0,0;
               1.48,  55,75,0     80,0,0;
               1.50,  31,75,0     130,0,0;
               1.49,  20,60,0     120,0,0;
               1.48,  0,44,0      -160,0,0;
               0,     23,36,0,    -70,0,0];

%% sine traj
x_max = 100;
num_sample = 50;
x = linspace(0, x_max, num_sample)';
y = 20 * sin(2 * pi .* x ./ 100);
tan_angle = atand(40 * pi / x_max * cos(2 * pi .* x ./ x_max));
psi = tan_angle - 90;

% plot(x, y);
% u = cosd(tan_angle);
% v = sind(tan_angle);
% quiver(x, y, u, v)

constraints(1:num_sample, :) = nan;

% ground speed
% constraints(1, 1) = 0;
% constraints(2:num_sample - 1, 1) = 1.5;
% constraints(num_sample, 1) = 0;
load("groundspeed.mat", "groundspeed");
constraints(:, 1) = groundspeed;

% waypoints
constraints(:, 2:4) = [x, y, zeros(num_sample, 1)];
constraints(:, 5:7) = [psi, zeros(num_sample, 2)];


%% set trajectory obj
trajectory = waypointTrajectory(constraints(:,2:4), ...
    GroundSpeed=constraints(:,1), ...
    Orientation=eul2rotm(deg2rad(constraints(:, 5:7))), ...
    ReferenceFrame='ENU', ...
    SampleRate=40);

% a table of specified constraints
tInfo = waypointInfo(trajectory);

%% calculate trajectory
% pos_log = zeros(8741, 3);
% orient_log = zeros(8741, 3, 3);
% vel_log = zeros(8741, 3);
% acc_log = zeros(8741, 3);
% angVel_log = zeros(8741, 3);
% velb_log = zeros(8741, 3);

% h = animatedline;
count = 1;
while ~isDone(trajectory)
   % [pos,orient(count),vel(count,:),acc(count,:),angVel(count,:)] = trajectory();

   [pos, orient, vel, acc, angVel] = trajectory();
   
   pos_log(count, :) = pos;
   orient_log(count, :, :) = orient;
   vel_log(count, :) = vel;
   acc_log(count, :) = acc;
   angVel_log(count, :) = angVel;

   velb = orient' * vel';
   velb_log(count, :) = velb';

   accb = orient' * acc';
   accb_log(count, :) = accb';

   angVelb = orient' * angVel';
   angVelb_log(count, :) = angVelb';

   % addpoints(h, p(1), p(2));
   % drawnow limitrate

   count = count + 1;
end
% drawnow

%% plot result
close all;
figure(1)
plot(tInfo.Waypoints(1,1),tInfo.Waypoints(1,2),"b*")
title("Position")
% axis([20,65,0,25])
xlabel("East")
ylabel("North")
grid on
daspect([1 1 1])
% xlim([0, 80])
% ylim([0, 80])
hold on

% figure(1)
plot(pos_log(:, 1),pos_log(:, 2),"b")
grid on

eulerAngles = rad2deg(rotm2eul(permute(orient_log, [2 3 1])));
% range = 2:100:8742;
range = 2:100:3978;
X(1:size(range)) = nan;
Y(1:size(range)) = nan;
U(1:size(range)) = nan;
V(1:size(range)) = nan;
cnt = 1;
for idx = range
    X(cnt) = pos_log(idx, 1);
    Y(cnt) = pos_log(idx, 2);
    U(cnt) = cosd(eulerAngles(idx, 1) + 90);
    V(cnt) = sind(eulerAngles(idx, 1) + 90);
    cnt = cnt + 1;
end
quiver(X, Y, U, V)

figure(2)
timeVector = 0:(1/trajectory.SampleRate):tInfo.TimeOfArrival(end);
% eulerAngles = eulerd([tInfo.Orientation{1};orient],"ZYX","frame");
plot(timeVector(2:end),eulerAngles(:,1), ...
     timeVector(2:end),eulerAngles(:,2), ...
     timeVector(2:end),eulerAngles(:,3));
title("Orientation Over Time")
legend("Rotation around Z-axis", ...
       "Rotation around Y-axis", ...
       "Rotation around X-axis", ...
       "Location","southwest")
xlabel("Time (seconds)")
ylabel("Rotation (degrees)")
grid on
hold on

figure(3)
plot(timeVector(2:end),vel_log(:,1), ...
     timeVector(2:end),vel_log(:,2), ...
     timeVector(2:end),vel_log(:,3));
title("Velocity Over Time")
legend("East","North","Up")
xlabel("Time (seconds)")
ylabel("Velocity (m/s)")
grid on
hold on

figure(4)
plot(timeVector(2:end),acc_log(:,1), ...
     timeVector(2:end),acc_log(:,2), ...
     timeVector(2:end),acc_log(:,3));
title("Acceleration Over Time")
legend("East","North","Up","Location","southwest")
xlabel("Time (seconds)")
ylabel("Acceleration (m/s^2)")
grid on

figure(5)
plot(timeVector(2:end),angVel_log(:,1), ...
     timeVector(2:end),angVel_log(:,2), ...
     timeVector(2:end),angVel_log(:,3));
title("Angular Velocity Over Time")
legend("East","North","Up")
xlabel("Time (seconds)")
ylabel("Angular Velocity (rad/s)")
grid on

figure(6)
plot(timeVector(2:end),velb_log(:,1), ...
     timeVector(2:end),velb_log(:,2), ...
     timeVector(2:end),velb_log(:,3));
title("Velocity In Body Frame Over Time")
legend("East","North","Up")
xlabel("Time (seconds)")
ylabel("Velocity (m/s)")
grid on

figure(7)
plot(timeVector(2:end),accb_log(:,1), ...
     timeVector(2:end),accb_log(:,2), ...
     timeVector(2:end),accb_log(:,3));
title("Acceleration In Body Frame Over Time")
legend("East","North","Up")
xlabel("Time (seconds)")
ylabel("Acceleration (m/s^2)")
grid on

figure(8)
plot(timeVector(2:end),angVelb_log(:,1), ...
     timeVector(2:end),angVelb_log(:,2), ...
     timeVector(2:end),angVelb_log(:,3));
title("Angular Velocity In Body Frame Over Time")
legend("East","North","Up")
xlabel("Time (seconds)")
ylabel("Angular Velocity (rad/s)")
grid on

% compare reconstructed orientation (yaw)
yaw = -38.5;
yaw_log(1:3978) = nan;
yaw_log(1) = yaw;
for i = 2:3978
    yaw = yaw - rad2deg(angVel_log(i, 3)) * 0.025;
    yaw_log(i) = yaw;
end
figure(2)
plot(timeVector(2:end), yaw_log)

% compare reconstructed velocity
vel_re = [0;0;0];
vel_re_log(1:3978, 1:3) = nan;
vel_re_log(1, :) = vel_re;
for i = 2:3978
    vel_re = vel_re + R3(deg2rad(yaw_log(i))) * accb_log(i, :)' * 0.025;
    vel_re_log(i, :) = vel_re;
end
figure(3)
plot(timeVector(2:end),vel_re_log(:,1), ...
     timeVector(2:end),vel_re_log(:,2), ...
     timeVector(2:end),vel_re_log(:,3));

% compare reconstructed position
pos_re = [0;0;0];
pos_re_log(1:3978, 1:3) = nan;
pos_re_log(1, :) = pos_re;
for i = 2:3978
    pos_re = pos_re + (vel_re_log(i-1, :) + vel_re_log(i, :))' * 0.5 * 0.025;
    pos_re_log(i, :) = pos_re;
end
figure(1)
plot(pos_re_log(:,1), pos_re_log(:,2));
