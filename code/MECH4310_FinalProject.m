%% MECH 4310 - Final Project
% Jonas Wagner
clear; close all;
addpath(genpath(pwd));
[filepath,~,~] = fileparts(mfilename('fullpath'));
fig_subfolder = strcat(filepath,filesep,'figs');
data_subfolder = strcat(filepath,filesep,'data');


% sim settings
dt = 0.1; tf = 5;
tspan = 0:dt:tf;


%% CT-Model (currently using kinnematic model)
% bikeModel_CT(x,u,params)
nx = 3;
nu = 2;
sys_fun = @bikeModel_CT;
x0 = zeros(nx,1);

% update_eq = @(x,u,dt) sys_fun(x,u).*dt;%ode45(@(t,x) sys_fun(x,u),dt,x0);


% constraints
% (max/min of U_set)
bounds.u_ub = [10; pi/3];
bounds.u_lb = [0; -pi/3];
bounds.u_dot_ub = [2; pi/10];
bounds.u_dot_lb = -[2;1].*bounds.u_dot_ub;

U_set = Polyhedron('lb',bounds.u_lb,'ub',bounds.u_ub); U_set.minHRep;
U_dot_set = Polyhedron('lb',bounds.u_dot_lb, 'ub',bounds.u_dot_ub); U_dot_set.minHRep;

% %% Initial Simulation
% u0 = [10; 0];%deg2rad(5)];
% u_fun = @(t) u0 + [0; deg2rad(15*sin(t))];
% 
% [t,x] = ode45(@(t,x) sys_fun(x,u_fun(t)),tspan,x0);
% 
% plot_vehicleModel(t,x',u_fun,bounds)


%% Cost Maps
% % example construction
% grid.sep = 0.1; %[m]
% grid.bounds.bwd = 5; %[m]
% grid.bounds.fwd = 20; 
% grid.lat = -grid.bounds.bwd:grid.sep:grid.bounds.fwd;
% grid.bounds.left = 10;
% grid.bounds.right = 10;
% grid.long = -grid.bounds.left:grid.sep:grid.bounds.right;
% [grid.pos.lat,grid.pos.long] = meshgrid(grid.long,grid.lat);
% 
% 
% clear cost
% cost = zeros(size(grid.pos.lat));
% % cost(grid.pos.lat < -2) = 5;
% % cost(grid.pos.lat > 2) = 5;
% % cost(grid.pos.long < -2) = 2;
% cost(grid.pos.long>5 & grid.pos.lat<-1) = 50;
% % cost(grid.pos.long>5 & grid.pos.lat<1) = 50;
% figure
% imshow(cost)



% load from cost-map png
costMap_filename = 'costmap1.png';
costMap_filename = strcat(data_subfolder,filesep,costMap_filename);
costMap = rescale(im2gray(imread(costMap_filename)));

% costMap_fun = @(x,y) 



%% MPC Controller Setup
dt_MPC = 1;
N = 15;

yalmip('clear'); clear('controller');
x_ = sdpvar(repmat(nx,1,N+1),ones(1,N+1));
u_ = sdpvar(repmat(nu,1,N),ones(1,N));
u_0 = sdpvar(nu,1);

constraints = []; objective = 0;
constraints = [constraints, U_dot_set.A*((u_{1}-u_0)/dt_MPC) <= U_dot_set.b];

for k = 1:N
    objective = objective + (x_{k}(3)-pi)^2; %idk...

    constraints = [constraints, x_{k+1} == update_eq(x_{k},u_{k},dt_MPC,sys_fun)];
    constraints = [constraints, U_set.A*u_{k} <= U_set.b];
    if k > 1
        constraints = [constraints, ...
            U_dot_set.A*((u_{k}-u_{k-1})/dt_MPC) <= U_dot_set.b];
    end
end

objective = objective + (x_{N+1}(3)-pi)^2; %<--- minimize these things...

% opts = sdpsettings('solver','ipopt');
opts = sdpsettings('solver','ipopt','debug',0,'verbose',0);
controller = optimizer(constraints, objective, opts, {x_{1},u_0}, [u_{1}]);






%% MPC controller testing

% sim settings
dt = 0.1; tf = 10;
tspan = 0:dt:tf;
x0 = zeros(nx,1);
u0 = zeros(nu,1);

timerVal = tic;
% Initial Conditions
X_{1} = x0; U_{1} = controller{x0,u0};
% Time Evolution
for k = 1:length(tspan)-1
    % Simulate Update Step
    [~,x_k] = ode45(@(t,x) sys_fun(x,U_{k}),tspan(k:k+1),X_{k});
    X_{k+1} = x_k(end,:)';
    % Next-timestep controller
    U_{k+1} = controller{X_{k+1},U_{k}};
    elapsedTime_{k} = toc(timerVal);
    fprintf('Finished k = %d... elapsed: t = %f\n',k,elapsedTime_{k});
end


x = [X_{:}];
u = [U_{:}];
plot_vehicleModel(tspan,x,u,bounds)







