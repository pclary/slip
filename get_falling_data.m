function get_falling_data()

codegen biped_sim -args {RobotState(), ControllerState(), RobotParams(), ControllerParams(), Terrain(), 0.1, 1e-3}

evalin('base', 'clear;biped_setup;');
Ts_dynamics = evalin('base', 'Ts_dynamics');
robot = evalin('base', 'robot');
footmass = robot.foot.mass;

env = Environment();
env.ground_data = zeros(403, 5);
env.ground_data(:, 1) = [-2e1-1; (-2e1:0.1:2e1)'; 2e1+1];
env.ground_data(:, 3) = 1e6;
env.ground_data(:, 4) = 1.5*2*sqrt(1e6*robot.foot.mass);
env.ground_data(:, 5) = 1;

% vis = BipedVisualization();
% vis.ground_data = env.ground_data;
% vis.setup(RobotState());

ntrials = 100000;
nsteps = 100;
ttfmax = 30;
tstep = 0.1;

tic;

data = [];
fprintf(repmat(' ', 1, 50));

fall = 0;
batchsize = 16;

for k = 1:ntrials/batchsize
    parfor j = 1:batchsize
        env = Environment();
        env.ground_data = zeros(403, 5);
        env.ground_data(:, 1) = [-2e1-1; (-2e1:0.1:2e1)'; 2e1+1];
        env.ground_data(:, 3) = 1e6;
        env.ground_data(:, 4) = 1.5*2*sqrt(1e6*footmass);
        env.ground_data(:, 5) = 1;
        
        % Start a new fixed rng sequence with each numbered trial
        rng((k - 1)*batchsize + j);
        
        % Randomize terrain
        h = rand()*0.03;
        ground_y = [-1e2; randn(401, 1)*h; -1e2];
        njogs = 10;
        ijogs = randi(numel(ground_y), 1, njogs);
        jogsize = rand(1, njogs) * 0.15 - 0.1;
        jogsize(ijogs < numel(ground_y) / 2) = jogsize(ijogs < numel(ground_y) / 2) + 0.2;
        jogs = zeros(size(ground_y));
        jogs(ijogs) = jogsize;
        jogs = cumsum(jogs);
        ground_y = ground_y + jogs;
        ground_y = ground_y - ground_y(env.ground_data(:, 1) == 0);
        env.ground_data(:, 2) = ground_y;
%         vis.ground_data = env.ground_data;
        
        % Reset robot state
        X = RobotState();
        X.body.y = X.body.y + ground_y(env.ground_data(:, 1) == 0);
        X.body.dx = randn() * 0.2;
        X.body.dy = randn() * 0.1;
        cstate = ControllerState();
        phase_offset = rand();
        cstate.phase.right = (cstate.phase.right + phase_offset) - floor(cstate.phase.right + phase_offset);
        cstate.phase.left = (cstate.phase.left + phase_offset) - floor(cstate.phase.left + phase_offset);
        cparams = ControllerParams();
        cparams.target_dx = randn() * 2;
        cparams.step_height = cparams.step_height + h;
        cparams.phase_rate = cparams.phase_rate + randn() * 0.05;
        cparams.energy_injection = cparams.energy_injection + randn() * 50;
%         vis.reset();
%         vis.step(X);
%         drawnow;
        
        newdata = rstate2vec(X);
        
        while true%vis.isAlive()
            % Add random kicks
            if randi(20) == 1
                X.body.dx = X.body.dx + randn() * 0.6;
                X.body.dy = X.body.dy + randn() * 0.3;
            end
            
            % Simulate
            terrain = env.getLocalTerrain(X.body.x);
            [X, cstate] = biped_sim_mex(X, cstate, robot, cparams, terrain, tstep, Ts_dynamics);
            newdata = [newdata, rstate2vec(X)];
            
%             % Update visualization
%                     try
%                         vis.step(X);
%                         drawnow;
%                     end
            
            % Check if crashed
            if X.body.y < 0 || abs(X.body.theta) > pi/2 || abs(X.right.theta - X.left.theta) > pi*0.8
                data = [data, [newdata; min(size(newdata, 2)-1:-1:0, ttfmax)]];
                fall = fall + 1;
                break;
            end
            
            % Check if end of trial
            if size(newdata, 2) == nsteps
                data = [data, [newdata(:, 1:end-ttfmax); ones(1, nsteps-ttfmax)*ttfmax]];
                break;
            end
        end
        
        
        %     if ~vis.isAlive()
        %         break;
        %     end
    end
    
    j = k*batchsize;
    fprintf([repmat('\b', 1, 50), '%5.1f %% [%7.1f s] [ETR %7.1f s] [%5.1f %% fall]'], ...
        j/ntrials*100, toc, toc*(1/(j/ntrials) - 1), fall/j*100);
end

fprintf('\n');
csvwrite('test.csv', data');

end


function Xv = rstate2vec(X)
Xv = [...
    X.body.x;
    X.body.y;
    X.body.theta;
    X.body.dx;
    X.body.dy;
    X.body.dtheta;
    X.right.l;
    X.right.l_eq;
    X.right.theta;
    X.right.theta_eq;
    X.right.dl;
    X.right.dl_eq;
    X.right.dtheta;
    X.right.dtheta_eq;
    X.left.l;
    X.left.l_eq;
    X.left.theta;
    X.left.theta_eq;
    X.left.dl;
    X.left.dl_eq;
    X.left.dtheta;
    X.left.dtheta_eq];
end
