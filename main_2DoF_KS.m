function main_2DoF_KS()
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
grid_min = [0; -pi/2; -20; -20]; % Lower corner of computation domain
grid_max = [pi; pi/2; 20; 20];    % Upper corner of computation domain
%N = [41; 41; 41; 41];         % Number of grid points per dimension
N = [15; 15; 15; 15];
pdDims = [1; 2];               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
R = 0.2;
theta = pi/4;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
data0 = shapeCylinder(g, [3;4], [theta; theta; 0; 0], R);
% also try shapeRectangleByCorners, shapeSphere, etc.

%% time vector
t0 = 0;
tMax = 5;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
uMax = 25;
dMax = 0.1;

% do dStep1 here

% control trying to min or max value function?
uMode = 'min';
% do dStep2 here
dMode = 'max';

%% Pack problem parameters

% Define dynamic system
% obj = DubinsCar(x, wMax, speed, dMax)
dCar = Arm4D_KS([0, 0, 0, 0], uMax, dMax, grid_min, grid_max); %do dStep3 here
% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
%do dStep4 here
schemeData.dMode = dMode;

%% If you have obstacles, compute them here
oR=0.2;
otheta = pi/8;
obstacles = shapeCylinder(g, [3;4], [otheta; otheta; 0; 0], oR);
%HJIextraArgs.obstacles = obstacles;

%% Compute value function

HJIextraArgs.visualize = false; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update


%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'none', HJIextraArgs);
csvwrite('4Dim_KS.csv',data);



%% Compute optimal trajectory from some initial state
if compTraj
  %pause
  
  %set the initial state
  xinit = [0, 0, 0, 0];
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data(:,:,:,:,end),xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    %disp(value)
    dCar.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.visualize = true; %show plot
    TrajextraArgs.fig_num = 2; %figure number
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0 0]; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,5);
    
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, dCar, TrajextraArgs);

v = VideoWriter('movie.mp4','MPEG-4');
% v = VideoWriter('movie.avi');
open(v);
%xy plot
iter2 = 1;
l = 5;
%limits = [0 12 0 12];
x = 0:0.2:l;

%Target
i=1;
dest_xx1 = zeros(41,1);
dest_yy1 = zeros(41,1);
dest_xx2 = zeros(41,1);
dest_yy2 = zeros(41,1);
R=0.2;

for theta1 = theta-R:0.01:theta+R
   root = sqrt(R^2-(theta1-theta)^2);
   theta2_minus = theta-root; 
   theta2_plus = theta+root; 
   dest_xx1(i) = l*cos(theta1)+l*cos(theta1+theta2_minus);
   dest_yy1(i) = l*sin(theta1)+l*sin(theta1+theta2_minus);
   dest_xx2(i) = l*cos(theta1)+l*cos(theta1+theta2_plus);
   dest_yy2(i) = l*sin(theta1)+l*sin(theta1+theta2_plus);
   i=i+1;
end
count_i = i-1;

%Obstacle
j=1;
dest_oxx1 = zeros(41,1);
dest_oyy1 = zeros(41,1);
dest_oxx2 = zeros(41,1);
dest_oyy2 = zeros(41,1);

for otheta1 = otheta-oR:0.01:otheta+oR
   oroot = sqrt(oR^2-(otheta1-otheta)^2);
   otheta2_minus = otheta-oroot; 
   otheta2_plus = otheta+oroot; 
   dest_oxx1(j) = l*cos(otheta1)+l*cos(otheta1+otheta2_minus);
   dest_oyy1(j) = l*sin(otheta1)+l*sin(otheta1+otheta2_minus);
   dest_oxx2(j) = l*cos(otheta1)+l*cos(otheta1+otheta2_plus);
   dest_oyy2(j) = l*sin(otheta1)+l*sin(otheta1+otheta2_plus);
   j=j+1;
end
count_oi = j-1;


figure

while iter2 <= length(traj_tau)
    xx1 = x*cos(traj(1, iter2));
    yy1 = x*sin(traj(1, iter2));
    xx2 = x*cos(traj(2, iter2)+traj(1, iter2))+l*cos(traj(1, iter2));
    yy2 = x*sin(traj(2, iter2)+traj(1, iter2))+l*sin(traj(1, iter2));
    plot(xx1, yy1, 'k-')
    
    hold on
    plot(xx2, yy2, 'k-')
    
    %Target plot
    for i=1:1:count_i
        plot(dest_xx1(i), dest_yy1(i), 'b.')
        plot(dest_xx2(i), dest_yy2(i), 'b.')
    end
    
    %Obstacle plot
    for i=1:1:count_oi
        plot(dest_oxx1(i), dest_oyy1(i), 'g.')
        plot(dest_oxx2(i), dest_oyy2(i), 'g.')   
    end

    %axis(limits)
    iter2 = iter2 + 1;
    drawnow
    %hold off

    frame = getframe(gcf); % アクティブなfigureをframeとして描きだす
    writeVideo(v,frame);
end
close(v); % とじる
  
  
  else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end

end
