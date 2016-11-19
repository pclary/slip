function cstate = ControllerState()

cstate.phase.right = 0;
cstate.phase.left  = 0.5;

cstate.foot_x_last.right = 0;
cstate.foot_x_last.left  = 0;

cstate.foot_x_target.right = 0;
cstate.foot_x_target.left  = 0;

cstate.body_ddx = 0;
cstate.body_dx_last = 0;
