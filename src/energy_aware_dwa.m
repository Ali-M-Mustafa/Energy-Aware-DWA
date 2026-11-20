% -------------------------------------------------------------------------
%
% File : energy_aware_dwa.m
%
% Description : Energy-aware DWA, eq. (16). Proposed method of Table III.
%               energy_aware_dwa(1) -> map 1,  energy_aware_dwa(2) -> map 2
%               energy_aware_dwa(map_id, delta) sets the energy weight
%               (default 0.5; delta = 0 removes the energy term)
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Editor : Omer Abubakr
%
% Adapted from DynamicWindowApproachSample.m,
% Copyright (c) 2014 Atsushi Sakai, Modified BSD License
% (https://github.com/AtsushiSakai/MATLABRobotics), as edited by Omer Abubakr.
%
% -------------------------------------------------------------------------

function [] = energy_aware_dwa(map_id, delta)
if nargin < 1
    map_id = 1;
end
if nargin < 2
    delta = 0.5;
end
 warning('off','all')
% close all;
pause on;

disp('Dynamic Window Approach sample program start!!')


%Obstacles list [x (m) y (m)], start and goal
[obstacle, start, goal] = get_map(map_id);
x=[start 0 0 0]';%initial state of the robot [x (m), y (m), yaw (Rad), v (m / s), omega (rad / s)]

% evalParam=[0.4,0.2,0.2,0.2,3];   %alpha,Beta and Gamma
% [heading, clearance, velocity, energy, PredictDT]
evalParam = [0.5, 0.5, 0.5, delta, 3];

obstacleR=0.175; %Radius of obstacle for collision determination 
global robot dt; dt=0.1;%time increment[s]


% Kinematic
%Maximum speed [m / s], maximum rotation speed [rad / s], maximum acceleration / deceleration [m / ss], 
%maximum adjustment head speed [rad / ss],
% speed resolution [m / s ], Turning speed resolution [rad / s]] 

% Kinematic=[0.6,toRadian(30), 0.2, toRadian(30),0.01,toRadian(1)];
Kinematic=[0.5,toRadian(180), 1, toRadian(180),0.02,toRadian(5)];

%Evaluation parameter of the function [heading, dist, velocity, PredictDT]
%predicted time difference PredictDT

wplot = [];
area=[-2 goal(1)+5 -2 goal(2)+5];%Simulation wide area [xmin xmax ymin ymax]

% Robot & Energy Parameters
robot.r    = 0.033;    % wheel radius [m]
robot.B    = 0.16;     % track width [m]
robot.KT   = 0.024;    % torque constant [N·m/A] (Table II rounds this to 0.02)
robot.gr   = 15;       % gear ratio right
robot.gl   = 15;       % gear ratio left
robot.eta  = 0.78;     % transmission efficiency
robot.re   = 0.74;     % coil resistance [Ω]
robot.mu   = 0.69;     % ground friction factor
robot.mass = 3.0;      % robot mass [kg] (example)
robot.g    = 9.81;     % gravity [m/s²]
% --- Metrics ---
E_total   = 0;    % total energy [J]
t_sim     = 0;    % simulated time [s]
path_len  = 0;    % executed path length [m]
E_hist    = [];   % energy per step [J]
t_hist    = [];   % time stamps [s]

%Simulation result
result.x=[];
E_array =[];
tic;    %Start stopwatch timer
%movcount=0;

%************######### Main loop ############################************
for i=1:5000
% #####################Dynamic Obsacles####################################
%      if(mod(i,10)==0)
%          for tz = 1:6
%              temp = rand(1,1);
%              if(temp>0.5)
%                  obstacle(tz ,1) = obstacle(tz ,1)-1;
%                  obstacle(tz ,2) = obstacle(tz ,1)+1;
%              else
%                  obstacle(tz ,1) = obstacle(tz ,1)+1;
%                  obstacle(tz ,2) = obstacle(tz ,1)-1;
%              end
%          end
%      end

%% #####################Dynamic Goal####################################

%      goal = goal + 0.04;
%      if(goal(1) > area(2) || goal(2) > area(4))
%         area(2) = area(2) + 1;
%         area(4) = area(4) + 1;
%      end
% #########################################################################
    
%% #####################Weight Adjusment####################################
     
%     dist = ObsDist(x,obstacle,obstacleR);
%     weights = wAdjust(dist,wfis);
%     
%     if(sum(weights)>1)
%         weights(1:3) = weights(1:3)/sum(weights);
%     end
% %     weights
%     evalParam = [weights(1) weights(2) weights(3) 2];
%     weights = evalParam(1:3);
%     wplot = [wplot; weights];

%% #########################################################################
    
    %Calculation of input value by DWA
    [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    if u(1) == 0 && u(2) == 0
        return;
    end
    
    x_prev = x;
    x=f(x,u);% move the robot to the next time step
    
    % --- Metrics update (executed control over this dt) ---
    E_step   = CalcEnergy(u(1),u(2));
    E_array  =[E_array; E_step];
    E_total  = E_total + E_step;
    t_sim    = t_sim + dt;
    path_len = path_len + hypot(x(1)-x_prev(1), x(2)-x_prev(2));
    E_hist   = [E_hist; E_step]; %#ok<AGROW>
    t_hist   = [t_hist; t_sim];  %#ok<AGROW>

    %Save simulation result
    result.x=[result.x; x'];
    
    %Goal judgment
    if norm(x(1:2)-goal')<0.5
        disp('Arrive Goal!!');break;
    end
    
    
    %====Animation====
    hold off;
    ArrowLength=2;%arrow length 
    %robot
    
    
    draw_robot(x(1),x(2));hold on;
%     quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'ok');hold on;
%     quiver(x(1),x(2),ArrowLength*cos(x(3)+pi/2),ArrowLength*sin(x(3)+pi/2),'ok');
%     quiver(x(1),x(2),ArrowLength*cos(x(3)-pi/2),ArrowLength*sin(x(3)-pi/2),'ok');
    
    plot(result.x(:,1),result.x(:,2),'-b');hold on;
    plot(result.x(1,1),result.x(1,2),'*m');hold on;
    plot(goal(:,1),goal(:,2),'*r');hold on;
    plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
    %search path display 
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5
            ind=1+(it-1)*5;
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
        end
    end
    axis(area);
    grid on;
%     title(result.x(size(result.x,1),3) + " = " + norm(goal(1:2)-x(1:2)'))
    title(sprintf('Energy-aware DWA, map %d', map_id))
    drawnow;
    %movcount=movcount+1;
    %mov(movcount) = getframe(gcf);% Get the animation frame
  
end
figure(2)
subplot(2,1,1)
plot(result.x(:,4));
grid;
title('Translational Velocity')
ylabel('m/s')
subplot(2,1,2)
plot(result.x(:,5));
grid;
title('Rotational Velocity')
ylabel('rad/s')

if ~isempty(wplot)
    figure(3)
    subplot(3,1,1);
    plot(wplot(:,1),'-g');
    legend('Alpha');
    grid;
    subplot(3,1,2);
    plot(wplot(:,2),'-r');
    legend('Beta');
    grid;
    subplot(3,1,3);
    plot(wplot(:,3));
    legend('Gamma');
    grid;
end

figure
plot(result.x(:,3));
toc
%Movie2avi (mov, 'movie.avi');
% ==== Final navigation metrics ====
fprintf('\n===== Navigation metrics =====\n');
fprintf('Path length        : %.3f m\n', path_len);
fprintf('Simulated time     : %.2f s (dt=%.3f)\n', t_sim, dt);
fprintf('Steps              : %d\n', numel(E_hist));
fprintf('Total energy       : %.3f J\n', E_total);
if t_sim>0
    fprintf('Average power      : %.3f W\n', E_total/t_sim);
end
tag = '';
if delta ~= 0.5
    tag = sprintf('_delta%g', delta);
end
PathXY = result.x(:,1:2);  % columns: [x y]
Tpath  = array2table(PathXY, 'VariableNames', {'x','y'});
writetable(Tpath, sprintf('E_map%d%s_xy.csv', map_id, tag));   % saves in current folder
Te  = array2table(E_array, 'VariableNames', {'e'});
writetable(Te, sprintf('E_map%d%s_e.csv', map_id, tag));   % saves in current folder


function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
%Function to calculate the input value by DWA
%Dynamic Window [vmin, vmax, ?min, ?max] Creating
Vr=CalcDynamicWindow(x,model);
%evaluation calculation function

[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end
%Normalization of each evaluation function
evalDB=NormalizeEval(evalDB);

%Calculating final evaluation value
feval=[];

for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:4)*evalDB(id,3:6)'];
end
evalDB=[evalDB feval];


[maxv,ind]=max(feval);%Calculate the index of the input value with the highest evaluation value
u=evalDB(ind,1:2)';%Return the input value with the higher evaluation value

function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
%
evalDB=[];
trajDB=[];

global dt;
evaldt  = evalParam(5);                 % prediction horizon [s]
nSteps  = max(1, ceil(evaldt/dt));      % number of integration steps

for vt=Vr(1):model(5):Vr(2)
    for ot=Vr(3):model(6):Vr(4)

        % Predict trajectory
        [xt,traj]=GenerateTrajectory(x,vt,ot,evaldt,model);

        % Scores / metrics
        heading   = CalcHeadingEval(xt,goal);

        % Clearance is evaluated over the whole predicted trajectory
        clearance = CalcDistEval(traj,ob,R);

        % Reject trajectories that come within 0.2 m of an obstacle
        if clearance < 0.2
            continue;
        end

        vel = abs(vt);

        % --- Energy cost ---
        % Predict total energy over horizon (J)
        E_pred = CalcEnergy(vt,ot) * nSteps;

        % Energy per metre (J/m), so slow candidates are not favoured just
        % because they use less energy per step
        travel = max(vel * evaldt, 1e-3);   % approximate traveled distance [m]
        Epm    = E_pred / travel;           % energy per meter [J/m] (cost)

        % Store: [v w heading clearance vel energy_cost]
        evalDB=[evalDB;[vt ot heading clearance vel Epm]];
        trajDB=[trajDB;traj];
    end
end

function EvalDB=NormalizeEval(EvalDB)
% Normalize heading, clearance, velocity as scores in [0,1]
EvalDB(:,3) = NormalizeByMax(EvalDB(:,3));   % heading (higher better)
EvalDB(:,4) = NormalizeByMax(EvalDB(:,4));   % clearance (higher better)
EvalDB(:,5) = NormalizeByMax(EvalDB(:,5));   % velocity (higher better)

% Energy column is a COST (J/m): lower is better.
% Convert to a SCORE: higher is better.
eCost = EvalDB(:,6);
eCost = MinMax01(eCost);         % 0=low cost, 1=high cost
EvalDB(:,6) = 1 - eCost;         % 1=best (lowest energy), 0=worst


function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
%function to create the trajectory data 
global dt;
time=0;
u=[vt;ot];%input value 
traj=x;%trajectory Data 
while time<=evaldt
    time=time+dt;%update of simulation time 
    x=f(x,u);%transition by motion model 
    traj=[traj x];
end

function dist=CalcDistEval(traj,ob,R)
% traj is 5xN (states in columns). We compute minimum clearance along it.

dist = inf;
pos = traj(1:2,:);  % 2xN

for io=1:size(ob,1)
    dx = pos(1,:) - ob(io,1);
    dy = pos(2,:) - ob(io,2);
    d  = sqrt(dx.^2 + dy.^2) - R; % clearance
    dist = min(dist, min(d));
end

if isinf(dist)
    dist = 0;
end


function heading=CalcHeadingEval(x,goal)
%Function to calculate the evaluation function of heading

theta=toDegree(x(3));%robot orientation 
goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));%the goal orientation

if goalTheta>theta
    targetTheta=goalTheta-theta;%azimuth difference up to the goal [deg]
else
    targetTheta=theta-goalTheta;%azimuth difference up to the goal [deg]
end

heading=180-targetTheta;

function Vr=CalcDynamicWindow(x,model)
%Compute DyamicWindow from model and current state 
global dt;
%Wind model by vehicle model
Vs=[0 model(1) -model(2) model(2)];

%Window by motion model 

Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];

%Final Dynamic Window calculation 
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
%[vmin,vmax,?min,?max]

function x = f(x, u)
% Motion Model
global dt;
 
F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0]; %to get x,y,yaw from x
 
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];

x= F*x+B*u;
if(x(3)>2*pi)
    x(3)=x(3)-2*pi;
elseif x(3)<-2*pi
    x(3)=x(3)+2*pi;
end

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;

% function dist = ObsDist(x,ob,R)
% %Obstacle distance for 5 regions
% dist=[3,3,3,3,3]; %e,d,c,b,a

% for io=1:length(ob(:,1))    
%     res=ob(io,:)-x(1:2)'; %Vector between the robot and the obstacle
%     thet=atan2(res(2),res(1)); % Angle of the vector
%     r=sqrt(res(1)^2+res(2)^2);  %Magnitude of the vector
%     if r<=3
%         if (abs(thet-x(3))+1.2566)>=-0.3141 && abs((thet-x(3))+1.2566)<=0.3142  %a Area
%             if dist(5)>r
%                 dist(5)=r;
%             end
%         elseif (abs(thet-x(3))+0.6283)>=-0.3141 && abs((thet-x(3))+0.6283)<=0.3142  %b Area
%             if dist(4)>r
%                 dist(4)=r;
%             end
%         elseif (abs(thet-x(3)))>=-0.3141 && abs((thet-x(3)))<=0.3142  %c Area
%             if dist(3)>r
%                 dist(3)=r;
%             end
%         elseif (abs(thet-x(3))-0.6283)>=-0.3141 && abs((thet-x(3))-0.6283)<=0.3142  %d Area
%             if dist(2)>r
%                 dist(2)=r;
%             end
%         elseif (abs(thet-x(3))-1.2566)>=-0.3141 && abs((thet-x(3))-1.2566)<=0.3142  %e Area
%             if dist(1)>r
%                 dist(1)=r;
%             end
%         end
%     end
% end
% % dist
% 
% function weight = wAdjust(dist,wfis)
% %Weight adjustment function
%     weight = evalfis(dist(1:5),wfis); %Apply fuzzy logic

function [] = draw_robot(x,y)
% Draw the robot as a circle of radius mob_r
mob_r=0.175;
th = 0:pi/50:2*pi;
xunit = mob_r * cos(th) + x;
yunit = mob_r * sin(th) + y;
h = plot(xunit, yunit,'-k');

function E = CalcEnergy(v, w)
    global robot dt;
    % Wheel speeds from (v, w), eq. (8)
    omega_l = v/robot.r - (robot.B*w)/(2*robot.r);
    omega_r = v/robot.r + (robot.B*w)/(2*robot.r);
    % Rolling/friction torque tau = mu*m*g*r*sgn(omega), eq. (11)
    C_f = robot.mu * robot.mass * robot.g * robot.r;
    tau_l = C_f * sign(omega_l);
    tau_r = C_f * sign(omega_r);
    % Coil current, eq. (13), and power, eq. (14)
    i_l = tau_l / (robot.KT * robot.gl * robot.eta);
    i_r = tau_r / (robot.KT * robot.gr * robot.eta);
    P_l = omega_l.*tau_l/robot.eta + (i_l.^2)*robot.re;
    P_r = omega_r.*tau_r/robot.eta + (i_r.^2)*robot.re;
    % Energy over one interval, eq. (15)
    E = (P_l + P_r) * dt;


function y = NormalizeByMax(x)
    m = max(x);
    if m < 1e-12
        y = zeros(size(x));
    else
        y = x ./ m;
    end


function y = MinMax01(x)
    xmin = min(x);
    xmax = max(x);
    if (xmax - xmin) < 1e-12
        y = zeros(size(x));
    else
        y = (x - xmin) ./ (xmax - xmin);
    end

