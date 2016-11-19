function X = RobotState()

X.body.x      = 0;
X.body.y      = 1;
X.body.theta  = 0;
X.body.dx     = 0;
X.body.dy     = 0;
X.body.dtheta = 0;

X.right.l         = 1;
X.right.l_eq      = 1;
X.right.theta     = 0;
X.right.theta_eq  = 0;
X.right.dl        = 0;
X.right.dl_eq     = 0;
X.right.dtheta    = 0;
X.right.dtheta_eq = 0;

X.left.l         = 1;
X.left.l_eq      = 1;
X.left.theta     = 0;
X.left.theta_eq  = 0;
X.left.dl        = 0;
X.left.dl_eq     = 0;
X.left.dtheta    = 0;
X.left.dtheta_eq = 0;
