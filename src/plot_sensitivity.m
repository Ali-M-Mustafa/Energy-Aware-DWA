% plot_sensitivity.m
% Trajectories and velocity profiles for the weight sensitivity analysis
% on map 3 (Fig. 5). Run sensitivity_analysis first.

[obstacle, start, goal] = get_map(3);

% Read CSV
T1 = readtable('E_map3_xy_equal.csv');   % columns: x, y
T2 = readtable('E_map3_xy_alpha09.csv');   % columns: x, y
T3 = readtable('E_map3_xy_beta09.csv');   % columns: x, y
T4 = readtable('E_map3_xy_gamma09.csv');   % columns: x, y
T5 = readtable('E_map3_xy_delta09.csv');   % columns: x, y
T6 = readtable('E_map3_xy_delta0.csv');   % columns: x, y

V1 = readtable('E_map3_v_equal.csv');
%V1 = V1(:);                         % make sure it's a column vector
steps1 = (1:height(V1))';
V2 = readtable('E_map3_v_alpha09.csv');
steps2 = (1:height(V2))';
V3 = readtable('E_map3_v_beta09.csv');
steps3 = (1:height(V3))';
V4 = readtable('E_map3_v_gamma09.csv');
steps4 = (1:height(V4))';
V5 = readtable('E_map3_v_delta09.csv');
steps5 = (1:height(V5))';
V6 = readtable('E_map3_v_delta0.csv');
steps6 = (1:height(V6))';

W1 = readtable('E_map3_w_equal.csv');
%V1 = V1(:);                         % make sure it's a column vector
step1 = (1:height(W1))';
W2 = readtable('E_map3_w_alpha09.csv');
step2 = (1:height(W2))';
W3 = readtable('E_map3_w_beta09.csv');
step3 = (1:height(W3))';
W4 = readtable('E_map3_w_gamma09.csv');
step4 = (1:height(W4))';
W5 = readtable('E_map3_w_delta09.csv');
step5 = (1:height(W5))';
W6 = readtable('E_map3_w_delta0.csv');
step6 = (1:height(W6))';

%% ===================== Professional Plots =====================

% ---- Path figure ----
figure('Color','w'); hold on;

% Obstacles (bigger, black)
hObs = scatter(obstacle(:,1), obstacle(:,2), 70, 'k', 'filled', ...
               'DisplayName','Obstacles');

% Paths
hP1 = plot(T6.x, T6.y, '-', 'LineWidth', 2.2, ...
           'Color', [0 0.4470 0.7410], 'DisplayName', sprintf('\\delta=%.1f, \\alpha=\\beta=\\gamma=%.1f', 0, 0.5));
hP2 = plot(T1.x, T1.y, '-', 'LineWidth', 2.2, ...
           'Color', [0.8500 0.3250 0.0980], 'DisplayName', sprintf('\\alpha=\\beta=\\gamma=\\delta=%.1f', 0.5));
hP3 = plot(T2.x, T2.y, '-', 'LineWidth', 2.2, ...
           'Color', [0.9290 0.6940 0.1250], 'DisplayName', sprintf('\\alpha=%.1f, \\beta=\\gamma=\\delta=%.1f', 0.9, 0.5));
hP4 = plot(T3.x, T3.y, '-', 'LineWidth', 2.2, ...
           'Color', [0.4940 0.1840 0.5560], 'DisplayName', sprintf('\\beta=%.1f, \\alpha=\\gamma=\\delta=%.1f', 0.9, 0.5));
hP5 = plot(T4.x, T4.y, '-', 'LineWidth', 2.2, ...
           'Color', [0.4660 0.6740 0.1880], 'DisplayName', sprintf('\\gamma=%.1f, \\alpha=\\beta=\\delta=%.1f', 0.9, 0.5));
hP6 = plot(T5.x, T5.y, '-', 'LineWidth', 2.2, ...
           'Color', [0.3010 0.7450 0.9330], 'DisplayName', sprintf('\\delta=%.1f, \\alpha=\\beta=\\gamma=%.1f', 0.9, 0.5));


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


legend([hStart hGoal hObs hP1 hP2 hP3 hP4 hP5 hP6], 'Location','bestoutside');
set(gca,'FontSize',12,'LineWidth',1.0);


% ---- Linear velocity figure ----
figure('Color','w'); hold on;

v1 = V1.v; 
v2 = V2.v;
v3 = V3.v;
v4 = V4.v;
v5 = V5.v;
v6 = V6.v;

hV1 = plot(steps6, v6, '-', 'LineWidth', 2.0, ...
           'Color', [0 0.4470 0.7410], 'DisplayName',sprintf('\\delta=%.1f, \\alpha=\\beta=\\gamma=%.1f', 0, 0.5));
hV2 = plot(steps1, v1, '-', 'LineWidth', 2.0, ...
           'Color', [0.8500 0.3250 0.0980], 'DisplayName',sprintf('\\alpha=\\beta=\\gamma=\\delta=%.1f', 0.5));
hV3 = plot(steps2, v2, '-', 'LineWidth', 2.0, ...
           'Color', [0.9290 0.6940 0.1250], 'DisplayName',sprintf('\\alpha=%.1f, \\beta=\\gamma=\\delta=%.1f', 0.9, 0.5));
hV4 = plot(steps3, v3, '-', 'LineWidth', 2.0, ...
           'Color', [0.4940 0.1840 0.5560], 'DisplayName',sprintf('\\beta=%.1f, \\alpha=\\gamma=\\delta=%.1f', 0.9, 0.5));
hV5 = plot(steps4, v4, '-', 'LineWidth', 2.0, ...
           'Color', [0.4660 0.6740 0.1880], 'DisplayName',sprintf('\\gamma=%.1f, \\alpha=\\beta=\\delta=%.1f', 0.9, 0.5));
hV6 = plot(steps5, v5, '-', 'LineWidth', 2.0, ...
           'Color', [0.3010 0.7450 0.9330], 'DisplayName',sprintf('\\delta=%.1f, \\alpha=\\beta=\\gamma=%.1f', 0.9, 0.5));

grid on; box on;

xlabel('Step','FontWeight','bold');
ylabel('Linear velocity per step','FontWeight','bold');


% Tight axis with margin
allE = [v1(:); v2(:); v3(:); v4(:); v5(:); v6(:)];
nmax = max([numel(v1), numel(v2), numel(v3), numel(v4), numel(v5), numel(v6)]);
xlim([1, nmax]);
%xlim([1, max(numel(v1), numel(v2), numel(v3), numel(v4), numel(v5), numel(v6))]);
ylim([min(allE)*0.95, max(allE)*1.05]);

legend('Location','best');
set(gca,'FontSize',12,'LineWidth',1.0);

% ---- Angular velocity figure ----
figure('Color','w'); hold on;
w1 = W1.w; 
w2 = W2.w;
w3 = W3.w;
w4 = W4.w;
w5 = W5.w;
w6 = W6.w;

hW1 = plot(step6, w6, '-', 'LineWidth', 2.0, ...
           'Color', [0 0.4470 0.7410], 'DisplayName',sprintf('\\delta=%.1f, \\alpha=\\beta=\\gamma=%.1f', 0, 0.5));
hW2 = plot(step1, w1, '-', 'LineWidth', 2.0, ...
           'Color', [0.8500 0.3250 0.0980], 'DisplayName',sprintf('\\alpha=\\beta=\\gamma=\\delta=%.1f', 0.5));
hW3 = plot(step2, w2, '-', 'LineWidth', 2.0, ...
           'Color', [0.9290 0.6940 0.1250], 'DisplayName',sprintf('\\alpha=%.1f, \\beta=\\gamma=\\delta=%.1f', 0.9, 0.5));
hW4 = plot(step3, w3, '-', 'LineWidth', 2.0, ...
           'Color', [0.4940 0.1840 0.5560], 'DisplayName',sprintf('\\beta=%.1f, \\alpha=\\gamma=\\delta=%.1f', 0.9, 0.5));
hW5 = plot(step4, w4, '-', 'LineWidth', 2.0, ...
           'Color', [0.4660 0.6740 0.1880], 'DisplayName',sprintf('\\gamma=%.1f, \\alpha=\\beta=\\delta=%.1f', 0.9, 0.5));
hW6 = plot(step5, w5, '-', 'LineWidth', 2.0, ...
           'Color', [0.3010 0.7450 0.9330], 'DisplayName',sprintf('\\delta=%.1f, \\alpha=\\beta=\\gamma=%.1f', 0.9, 0.5));

grid on; box on;

xlabel('Step','FontWeight','bold');
ylabel('Angular velocity per step','FontWeight','bold');


% Tight axis with margin
allW = [w1(:); w2(:); w3(:); w4(:); w5(:); w6(:)];
nmax = max([numel(w1), numel(w2), numel(w3), numel(w4), numel(w5), numel(w6)]);
xlim([1, nmax]);
%xlim([1, max(numel(v1), numel(v2), numel(v3), numel(v4), numel(v5), numel(v6))]);
ylim([min(allW)*0.95, max(allW)*1.05]);

legend('Location','best');
set(gca,'FontSize',12,'LineWidth',1.0);