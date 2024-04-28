clc;clear;close all;

load("multibox-test-cond&imu.mat");
load("multibox-test-single-ds10.mat");
figure(1);
subplot(211);
plot(cond_tt);
c0 = cond_tt(50:100);
c1 = cond_tt(240:290);
c2 = cond_tt(400:450);
c3 = cond_tt(550:600);
c4 = cond_tt(850:900);

fprintf("Mean: %d, %d, %d, %d, %d \n", mean(c0), mean(c1), mean(c2), mean(c3), mean(c4));

% ylim([0, 10000]);
legend(["cond-T"]);

% subplot(312);
% plot(cond_rr);
% % ylim([0, 10000]);
% legend(["cond-R"]);


% load IMU to view state.
subplot(212);
plot(accelX, 'r', 'DisplayName', 'Accel X');
hold on;
plot(accelY, 'g', 'DisplayName', 'Accel Y');
plot(accelZ, 'b', 'DisplayName', 'Accel Z');
title('IMU Accelerations');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend show;



%% 
% load("heigt=0.mat");
% 
% c0_r = cond_rr;
% c0_t = cond_tt;
% 
% load("heigt=300.mat");
% c300_r = cond_rr;
% c300_t = cond_tt;
% 
% N = min(length(c0_r), length(c300_r));
% 
% 
% figure;
% 
% subplot(211);
% plot(c0_r); hold on;
% plot(c300_r);
% legend(["r0", "r300"]);
% 
% subplot(212); hold on;
% plot(c0_t); hold on;
% plot(c300_t);
% legend(["t0", "t300"]);
% hold off;


%% load IMU
% Load the ROS bag file
% test_file = "C:\Users\larrydong\Desktop\multi-box.bag";
% bag = rosbag(test_file);
% 
% % Select the IMU topic
% imuTopic = select(bag, 'Topic', '/ouster/imu');
% 
% % Read all IMU messages
% imuMsgs = readMessages(imuTopic);
% 
% % Initialize arrays to store IMU data
% numMessages = length(imuMsgs);
% accelX = zeros(numMessages, 1);
% accelY = zeros(numMessages, 1);
% accelZ = zeros(numMessages, 1);
% angularX = zeros(numMessages, 1);
% angularY = zeros(numMessages, 1);
% angularZ = zeros(numMessages, 1);
% timestamps = zeros(numMessages, 1);
% 
% % Extract data from each message
% for i = 1:numMessages
%     accelX(i) = imuMsgs{i}.LinearAcceleration.X;
%     accelY(i) = imuMsgs{i}.LinearAcceleration.Y;
%     accelZ(i) = imuMsgs{i}.LinearAcceleration.Z;
%     angularX(i) = imuMsgs{i}.AngularVelocity.X;
%     angularY(i) = imuMsgs{i}.AngularVelocity.Y;
%     angularZ(i) = imuMsgs{i}.AngularVelocity.Z;
%     timestamps(i) = seconds(imuMsgs{i}.Header.Stamp);
% end
% 
% % Normalize timestamps to start from zero
% timestamps = timestamps - timestamps(1);



