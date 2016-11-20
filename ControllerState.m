function cstate = ControllerState()

cstate.phase.right = 0;
cstate.phase.left  = 0.5;

cstate.dfilter_l_right.last    = 0;
cstate.dfilter_l_right.dtarget = 0;

cstate.dfilter_l_left.last    = 0;
cstate.dfilter_l_left.dtarget = 0;

cstate.dfilter_theta_right.last    = 0;
cstate.dfilter_theta_right.dtarget = 0;

cstate.dfilter_theta_left.last    = 0;
cstate.dfilter_theta_left.dtarget = 0;
