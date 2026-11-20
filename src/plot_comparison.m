function plot_comparison(map_id)
% plot_comparison.m
% Path and energy-per-step comparison of classical vs. energy-aware DWA
% (Figs. 3-4). Run classical_dwa(map_id) and energy_aware_dwa(map_id) first.
if nargin < 1
    map_id = 1;
end

[obstacle, start, goal] = get_map(map_id);

% Read CSV
T1 = readtable(sprintf('noE_map%d_xy.csv', map_id));   % columns: x, y
T2 = readtable(sprintf('E_map%d_xy.csv', map_id));   % columns: x, y

E1 = readtable(sprintf('noE_map%d_e.csv', map_id));
%E1 = E1(:);                         % make sure it's a column vector
steps1 = (1:height(E1))';
E2 = readtable(sprintf('E_map%d_e.csv', map_id));
steps2 = (1:height(E2))';

%% ===================== Professional Plots =====================

% ---- Path figure ----
figure('Color','w'); hold on;

% Obstacles (bigger, black)
hObs = scatter(obstacle(:,1), obstacle(:,2), 70, 'k', 'filled', ...
               'DisplayName','Obstacles');

% Paths (ONLY 2 colors)
hP1 = plot(T1.x, T1.y, '-', 'LineWidth', 2.2, ...
           'Color', [0 0.4470 0.7410], 'DisplayName','DWA (No Energy)');
hP2 = plot(T2.x, T2.y, '-', 'LineWidth', 2.2, ...
           'Color', [0.8500 0.3250 0.0980], 'DisplayName','Energy-aware DWA');

% Start (green, clear)
hStart = scatter(start(1), start(2), 120, 'g', 'filled', ...
                 'MarkerEdgeColor','k', 'LineWidth', 1.0, ...
                 'DisplayName','Start');

% Goal (red, clear icon)
hGoal = scatter(goal(1), goal(2), 140, 'r', 'p', 'filled', ...
                'MarkerEdgeColor','k', 'LineWidth', 1.0, ...
                'DisplayName','Goal');

% Axis formatting
axis equal;
grid on; box on;

% Nice limits with margin
allx = [start(1); goal(1); obstacle(:,1); T1.x; T2.x];
ally = [start(2); goal(2); obstacle(:,2); T1.y; T2.y];
marg = 0.8;
xlim([min(allx)-marg, max(allx)+marg]);
ylim([min(ally)-marg, max(ally)+marg]);

xlabel('X (m)','FontWeight','bold');
ylabel('Y (m)','FontWeight','bold');


legend([hStart hGoal hObs hP1 hP2], 'Location','bestoutside');
set(gca,'FontSize',12,'LineWidth',1.0);


% ---- Energy figure (Energy vs Steps) ----
figure('Color','w'); hold on;

% Energy per step (column "e" of the CSV files)
e1 = E1.e; 
e2 = E2.e;

hE1 = plot(steps1, e1, '-', 'LineWidth', 2.0, ...
           'Color', [0 0.4470 0.7410], 'DisplayName','No Energy');
hE2 = plot(steps2, e2, '-', 'LineWidth', 2.0, ...
           'Color', [0.8500 0.3250 0.0980], 'DisplayName','Energy-aware');

grid on; box on;

xlabel('Step','FontWeight','bold');
ylabel('Energy per step (J)','FontWeight','bold');


% Tight axis with margin
allE = [e1(:); e2(:)];
xlim([1, max(numel(e1), numel(e2))]);
ylim([min(allE)*0.95, max(allE)*1.05]);

legend('Location','best');
set(gca,'FontSize',12,'LineWidth',1.0);
end
