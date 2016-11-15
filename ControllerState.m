function s = ControllerState()

s.phase.right = 0;
s.phase.left  = 0.5;

s.foot_x_last.right = 0;
s.foot_x_last.left  = 0;

s.foot_x_target.right = 0;
s.foot_x_target.left  = 0;

s.body_ddx = 0;
s.body_dx_last = 0;
