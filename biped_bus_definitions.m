function biped_bus_definitions() 
% BIPED_BUS_DEFINITIONS initializes a set of bus objects in the MATLAB base workspace 

make_bus(RobotState, 'robot_state_bus', {'body_state_bus', 'leg_state_bus', 'leg_state_bus'});
make_bus(ControllerState, 'controller_state_bus', {'rl_bus', 'rl_bus', 'rl_bus'});
make_bus(RobotParams, 'robot_params_bus', {'body_params_bus', 'foot_params_bus', 'dof_params_bus', 'motor_params_bus', 'hardstop_params_bus', 'dof_params_bus', 'ground_params_bus'});
make_bus(ControllerParams, 'controller_params_bus', {'pd_bus', 'pd_bus', 'feedforward_bus', 'feedforward_bus', 'pd_bus'});
make_bus(Control, 'control_bus', {'leg_control_bus', 'leg_control_bus'});
make_bus(ExternalForces, 'external_force_bus', {'body_force_bus', 'foot_force_bus', 'foot_force_bus'});


function bus_names = make_bus(s, name, bus_names)

fn = fieldnames(s);
j = 1;
for i = 1:numel(fn)
    e = s.(fn{i});
    elems(i) = Simulink.BusElement;
    elems(i).Name = fn{i};
    elems(i).Dimensions = size(e);
    switch class(e)
        case 'struct'
            bus_name = bus_names{1};
            elems(i).DataType = ['Bus: ' bus_name];
            bus_names = bus_names(2:end);
            if ~evalin('base', ['exist(''' bus_name ''', ''var'') && isa(' bus_name ', ''Simulink.Bus'')'])
                bus_names = make_bus(e, bus_name, bus_names);
            end
        otherwise
            elems(i).DataType = class(e);
    end
end

bus = Simulink.Bus;
bus.Elements = elems;
assignin('base', name, bus);
