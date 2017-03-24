function test_terrain()

dists = [];
times = [];
realtimes = [];
rd = [];
ts = [];

delete(gcp('nocreate'))
parpool(3);

try
    load test_data.mat
end

npar = 16;

tic
j = 0;

while true
    
    ii = randperm(npar);
    for i = 1:npar
        j = mod(ii(i) - 1, 4) + 1;
        k = ceil(ii(i)/8);
        rd = [rd; j];
        ts = [ts; 1 + (k-1)*3];
        in(i) = Simulink.SimulationInput('biped_sim_interactive');
        in(i) = in(i).setVariable('rollout_depth', rd(end));
        in(i) = in(i).setVariable('transition_samples', ts(end));
        in(i) = in(i).setVariable('env', Environment(generate_environment_gapsteps()));
        in(i) = in(i).setModelParameter('ReturnWorkspaceOutputs', 'on');
        in(i) = in(i).setModelParameter('StopTime', sprintf('%f', 100 / ts(end)));
    end
    
    out = parsim(in, 'ShowProgress', true);
    toc
    
    for i = 1:npar
        if out(i).ErrorMessage
            disp(out(i).ErrorMessage);
        end
        x = out(i).X.body.x.Data(end);
        t = out(i).X.body.x.Time(end);
        rt = out(i).SimulationMetadata.TimingInfo.ExecutionElapsedWallTime;
        dists = [dists; x];
        times = [times; t];
        realtimes = [realtimes; rt];
    end
    
    fprintf('%d data points\n', size(dists, 1));
    save test_data.mat dists times realtimes rd ts
    
%     j = j + 1;
%     if j > 50
%         delete(gcp('nocreate'))
%         j = 0;
%     end
end

