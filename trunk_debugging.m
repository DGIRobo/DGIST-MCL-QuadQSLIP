%% Running Mujoco Simulation

clear all
clc
close all
system('./run_linux')
%% Getting Mujoco Simulation Data

Trunk_data = './data/data_trunk.csv';

Trunk_T = readtable(Trunk_data);

Trunk_VariableNames = Trunk_T.Properties.VariableNames;

Trunk_Arr = table2array(Trunk_T);

[Trunk_m,Trunk_n] = size(Trunk_Arr);
%% Plot for FL Leg (Front Left Leg)

figure(1);
subplot(3,2,1);
plot(Trunk_Arr(:,1), Trunk_Arr(:,2), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(Trunk_Arr(:,1), Trunk_Arr(:,5), 'Color', 'r', 'LineWidth', 3);
hold on;
plot(Trunk_Arr(:,1), Trunk_Arr(:,8), 'Color', 'g', 'LineWidth', 3);
xlabel('time (sec)');
ylabel('Trunk Height (m)');
legend('reference', 'estimation', 'real')

subplot(3,2,2);
plot(Trunk_Arr(:,1), Trunk_Arr(:,2)-Trunk_Arr(:,5), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(Trunk_Arr(:,1), Trunk_Arr(:,8)-Trunk_Arr(:,5), 'Color', 'r', 'LineWidth', 3);
xlabel('time (sec)');
ylabel('Trunk Height (m)');
legend('control error', 'estimation error')

subplot(3,2,3);
plot(Trunk_Arr(:,1), Trunk_Arr(:,3), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(Trunk_Arr(:,1), Trunk_Arr(:,6), 'Color', 'r', 'LineWidth', 3);
hold on;
plot(Trunk_Arr(:,1), Trunk_Arr(:,9), 'Color', 'g', 'LineWidth', 3);
xlabel('time (sec)');
ylabel('Trunk Roll (rad)');
legend('reference', 'estimation', 'real')

subplot(3,2,4);
plot(Trunk_Arr(:,1), Trunk_Arr(:,3)-Trunk_Arr(:,6), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(Trunk_Arr(:,1), Trunk_Arr(:,9)-Trunk_Arr(:,6), 'Color', 'r', 'LineWidth', 3);
xlabel('time (sec)');
ylabel('Trunk Roll (rad)');
legend('control error', 'estimation error')

subplot(3,2,5);
plot(Trunk_Arr(:,1), Trunk_Arr(:,4), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(Trunk_Arr(:,1), Trunk_Arr(:,7), 'Color', 'r', 'LineWidth', 3);
hold on;
plot(Trunk_Arr(:,1), Trunk_Arr(:,10), 'Color', 'g', 'LineWidth', 3);
xlabel('time (sec)');
ylabel('Trunk Pitch (rad)');
legend('reference', 'estimation', 'real')

subplot(3,2,6);
plot(Trunk_Arr(:,1), Trunk_Arr(:,4)-Trunk_Arr(:,7), 'Color', 'b', 'LineWidth', 3);
hold on;
plot(Trunk_Arr(:,1), Trunk_Arr(:,10)-Trunk_Arr(:,7), 'Color', 'r', 'LineWidth', 3);
xlabel('time (sec)');
ylabel('Trunk Pitch (rad)');
legend('control error', 'estimation error')