function gen_data()

scores_total = [];
rs_total = [];
cs_total = [];
tr_total = [];

try
    load planner_data.mat
end

npar = 16;

tic

while true
    
    for i = 1:npar
        in(i) = Simulink.SimulationInput('biped_sim_interactive');
        in(i) = in(i).setVariable('env', Environment(generate_environment_holes()));
        in(i) = in(i).setModelParameter('ReturnWorkspaceOutputs', 'on');
    end
    
    out = parsim(in, 'ShowProgress', true);
    toc
    
    for i = 1:npar
        scores_total = [scores_total; out(i).scores(2:end, :)];
        rs_total = [rs_total; out(i).rs(2:end, :)];
        cs_total = [cs_total; out(i).cs(2:end, :)];
        tr_total = [tr_total; out(i).tr(2:end, :)];
    end
    
    fprintf('%d data points\n', size(scores_total, 1));
    save planner_data.mat scores_total rs_total cs_total tr_total
end

