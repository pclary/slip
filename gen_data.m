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
j = 0;

while true
    
    for i = 1:npar
        in(i) = Simulink.SimulationInput('biped_sim_interactive');
        in(i) = in(i).setVariable('env', Environment(generate_environment_holes()));
        in(i) = in(i).setModelParameter('ReturnWorkspaceOutputs', 'on');
    end
    
    out = parsim(in, 'ShowProgress', true);
    toc
    
    for i = 1:npar
        if out(i).ErrorMessage
            disp(out(i).ErrorMessage);
        end
        scores_total = [scores_total; out(i).scores(4:end-3, :)];
        rs_total = [rs_total; out(i).rs(4:end-3, :)];
        cs_total = [cs_total; out(i).cs(4:end-3, :)];
        tr_total = [tr_total; out(i).tr(4:end-3, :)];
    end
    
    fprintf('%d data points\n', size(scores_total, 1));
    save planner_data.mat scores_total rs_total cs_total tr_total
    
%     j = j + 1;
%     if j > 50
%         delete(gcp('nocreate'))
%         j = 0;
%     end
end

