% Plot Roll
figure;
subplot(3,1,1);
plot(time_log, roll_log, 'b', 'DisplayName', 'Roll'); hold on;
plot(time_log, roll_des_log, 'r--', 'DisplayName', 'Roll Desired');
xlabel('Time (s)');
ylabel('Roll (rad)');
title('Roll vs Roll Desired');
legend;

% Plot Pitch
subplot(3,1,2);
plot(time_log, pitch_log, 'b', 'DisplayName', 'Pitch'); hold on;
plot(time_log, pitch_des_log, 'r--', 'DisplayName', 'Pitch Desired');
xlabel('Time (s)');
ylabel('Pitch (rad)');
title('Pitch vs Pitch Desired');
legend;

% Plot Yaw
subplot(3,1,3);
plot(time_log, yaw_log, 'b', 'DisplayName', 'Yaw'); hold on;
plot(time_log, yaw_des_log, 'r--', 'DisplayName', 'Yaw Desired');
xlabel('Time (s)');
ylabel('Yaw (rad)');
title('Yaw vs Yaw Desired');
legend;

