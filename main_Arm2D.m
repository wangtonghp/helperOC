function main_Arm2D()
% 1. Run Backward Reachable Set (BRS) with a goal
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = false <-- no trajectory
% 2. Run BRS with goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = true <-- compute optimal trajectory
% 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'zero' <-- Tube (not set)
%     compTraj = true <-- compute optimal trajectory
% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your DubinsCar
%     dStep4: add dMode to schemeData
% 5. Change to an avoid BRT rather than a goal BRT
%     uMode = 'max' <-- avoid
%     dMode = 'min' <-- opposite of uMode
%     minWith = 'zero' <-- Tube (not set)
%     compTraj = false <-- no trajectory
% 6. Change to a Forward Reachable Tube (FRT)
%     add schemeData.tMode = 'forward'
%     note: now having uMode = 'max' essentially says "see how far I can
%     reach"
% 7. Add obstacles
%     add the following code:
%     obstacles = shapeCylinder(g, 3, [-1.5; 1.5; 0], 0.75);
%     HJIextraArgs.obstacles = obstacles;
% 8. Add random disturbance (white noise)
%     add the following code:
%     HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];

%% Should we compute the trajectory?
compTraj = true;

%% Grid
grid_min = [-pi; -3;]; % Lower corner of computation domain
grid_max = [pi; 3];    % Upper corner of computation domain
N = [41; 41];         % Number of grid points per dimension
pdDims = 1;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
R = 0.2;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
data0 = shapeCylinder(g, 3, [pi/2; 0], R);
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = 2;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
u_max = 1;
d_max = 0.1;
% do dStep1 here

% control trying to min or max value function?
uMode = 'min';
dMode = 'max';
% do dStep2 here


%% Pack problem parameters

% Define dynamic system
% obj = DubinsCar(x, wMax, speed, dMax)
dCar = Arm2D([0, 0], u_max, d_max); %do dStep3 here

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;
%do dStep4 here


%% If you have obstacles, compute them here
obstacles = shapeCylinder(g, 2, 1, 0.1);
%HJIextraArgs.obstacles = obstacles;

%% Compute value function

HJIextraArgs.visualize = false; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'none', HJIextraArgs);
csvwrite('Arm2D.csv',data);

%% Compute optimal trajectory from some initial state
if compTraj
  %pause
  
  %set the initial state
  xinit = [0, 0];
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data(:,:,end),xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    
    dCar.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.visualize = false; %show plot
    TrajextraArgs.fig_num = 2; %figure number
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1]; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,3);
    
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, dCar, TrajextraArgs);

  
v = VideoWriter('Arm2D.mp4','MPEG-4'); % MPEG4����PowerPoint2010�ɓ\���Ă������Ȃ�
% v = VideoWriter('movie.avi');
open(v);
%xy plot
iter2 = 1;
l = 5;
limits = [-6 6 -6 6];
x = 0:0.2:l;
dest_area = R;
dest_theta = pi/2-dest_area:0.01:pi/2+dest_area;
figure

while iter2 <= length(traj_tau)
    xx = x*cos(traj(1, iter2));
    yy = x*sin(traj(1, iter2));
    oxx = l*cos(1);
    oyy = l*sin(1);
    dest_xx = l*cos(dest_theta);
    dest_yy = l*sin(dest_theta);
    plot(dest_xx, dest_yy, 'b.')
    hold on
    %plot(oxx, oyy, 'go')
    plot(xx, yy, 'k-')
    axis(limits)
    iter2 = iter2 + 1;
    drawnow
    %hold off

    frame = getframe(gcf); % �A�N�e�B�u��figure��frame�Ƃ��ĕ`������
    writeVideo(v,frame);
end
close(v); % �Ƃ���

  else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end

end
