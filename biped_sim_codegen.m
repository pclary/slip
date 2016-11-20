% BIPED_SIM_CODEGEN   Generate static library ControllerParams from biped_sim,
%  ControllerParams, ControllerState, RobotParams, RobotState.
% 
% Script generated from project 'biped_sim.prj' on 19-Nov-2016.
% 
% See also CODER, CODER.CONFIG, CODER.TYPEOF, CODEGEN.

%% Create configuration object of class 'coder.EmbeddedCodeConfig'.
cfg = coder.config('lib','ecoder',true);
cfg.IncludeTerminateFcn = false;
cfg.ConvertIfToSwitch = true;
cfg.EnableStrengthReduction = true;
cfg.EnableVariableSizing = false;
cfg.SaturateOnIntegerOverflow = false;
cfg.FilePartitionMethod = 'SingleFile';
cfg.InlineThreshold = 1000000;
cfg.InlineThresholdMax = 1000000;
cfg.InlineStackLimit = 1000000;
cfg.EnableOpenMP = false;
cfg.GenCodeOnly = true;
cfg.BuildConfiguration = 'Faster Runs';
cfg.TargetLangStandard = 'C99 (ISO)';
cfg.CodeReplacementLibrary = 'GNU C99 extensions';
cfg.SupportNonFinite = false;

%% Define argument types for entry-point 'biped_sim'.
ARGS = cell(6,1);
ARGS{1} = cell(7,1);

ARGS{1}{1} = RobotState();
ARGS{1}{1}.body = coder.cstructname(ARGS{1}{1}.body,'body_state_t');
ARGS{1}{1}.right = coder.cstructname(ARGS{1}{1}.right,'leg_state_t'); 
ARGS{1}{1}.left = coder.cstructname(ARGS{1}{1}.left,'leg_state_t');
ARGS{1}{1} = coder.cstructname(ARGS{1}{1},'robot_state_t');

ARGS{1}{2} = ControllerState();
ARGS{1}{2}.phase = coder.cstructname(ARGS{1}{2}.phase,'rl_t');
ARGS{1}{2}.dfilter_l_right = coder.cstructname(ARGS{1}{2}.dfilter_l_right,'dfilter_t');
ARGS{1}{2}.dfilter_l_left = coder.cstructname(ARGS{1}{2}.dfilter_l_left,'dfilter_t');
ARGS{1}{2}.dfilter_theta_right = coder.cstructname(ARGS{1}{2}.dfilter_theta_right,'dfilter_t');
ARGS{1}{2}.dfilter_theta_left = coder.cstructname(ARGS{1}{2}.dfilter_theta_left,'dfilter_t');
ARGS{1}{2} = coder.cstructname(ARGS{1}{2},'controller_state_t');

ARGS{1}{3} = RobotParams();
ARGS{1}{3}.body = coder.cstructname(ARGS{1}{3}.body,'body_params_t');
ARGS{1}{3}.foot = coder.cstructname(ARGS{1}{3}.foot,'foot_params_t');
ARGS{1}{3}.length.motor = coder.cstructname(ARGS{1}{3}.length.motor,'motor_params_t');
ARGS{1}{3}.length.hardstop = coder.cstructname(ARGS{1}{3}.length.hardstop,'hardstop_params_t');
ARGS{1}{3}.length = coder.cstructname(ARGS{1}{3}.length,'dof_params_t');
ARGS{1}{3}.angle.motor = coder.cstructname(ARGS{1}{3}.angle.motor,'motor_params_t');
ARGS{1}{3}.angle.hardstop = coder.cstructname(ARGS{1}{3}.angle.hardstop,'hardstop_params_t');
ARGS{1}{3}.angle = coder.cstructname(ARGS{1}{3}.angle,'dof_params_t');
ARGS{1}{3}.ground = coder.cstructname(ARGS{1}{3}.ground,'ground_params_t');
ARGS{1}{3} = coder.cstructname(ARGS{1}{3},'robot_params_t');

ARGS{1}{4} = ControllerParams();
ARGS{1}{4} = coder.typeof(ARGS{1}{4});
ARGS{1}{4} = coder.cstructname(ARGS{1}{4},'controller_params_t');

ARGS{1}{5} = Terrain();
ARGS{1}{5} = coder.cstructname(ARGS{1}{5},'terrain_t');

ARGS{1}{6} = coder.typeof(0);
ARGS{1}{7} = coder.typeof(0);

%% Invoke MATLAB Coder.
codegen -config cfg -o biped_sim biped_sim -args ARGS{1} ControllerParams ControllerState RobotParams RobotState Terrain
