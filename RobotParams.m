function robot = RobotParams()

robot.gravity      = 9.81;
robot.body.mass    = 30;
robot.body.inertia = 0.3;
robot.foot.mass    = 0.4;

robot.length.stiffness      = 1e4;
robot.length.damping        = 1e2;
robot.length.motor.inertia  = 1e-3;
robot.length.motor.damping  = 1e-1;
robot.length.motor.ratio    = 48;
robot.length.motor.torque   = 12;
robot.length.hardstop.min   = 0.3;
robot.length.hardstop.max   = 1;
robot.length.hardstop.kp    = 4e3;
robot.length.hardstop.kd    = 4e1;
robot.length.hardstop.dfade = 1e-2;
robot.length.hardstop.fmax  = 1e5;

robot.angle.stiffness      = 1e4;
robot.angle.damping        = 1e2;
robot.angle.motor.inertia  = 1e-3;
robot.angle.motor.damping  = 1e-1;
robot.angle.motor.ratio    = 16;
robot.angle.motor.torque   = 12;
robot.angle.hardstop.min   = -1.5;
robot.angle.hardstop.max   = 1.5;
robot.angle.hardstop.kp    = 1e3;
robot.angle.hardstop.kd    = 1e1;
robot.angle.hardstop.dfade = 1e-2;
robot.angle.hardstop.fmax  = 1e3;

robot.ground.damping_depth = 1e-3;
robot.ground.slip_ramp     = 1e-4;
