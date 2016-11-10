function s = ControllerState()

s = struct();
s.phase = struct('right', 0, 'left', 0.5);
s.X_laststep = RobotState();
s.footstep_target_right = 0;
s.footstep_target_left = 0;
s.angle_target_right_last = 0;
s.angle_target_left_last = 0;
