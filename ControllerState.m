function cstate = ControllerState()

offset = rand() * 0.5;

cstate.right.phase = offset;
cstate.right.foot_x_last = 0;
cstate.right.foot_x_target = 0;

cstate.left.phase = 0.5 + offset;
cstate.left.foot_x_last = 0;
cstate.left.foot_x_target = 0;

cstate.body_ddx = 0;
cstate.body_dx_last = 0;
