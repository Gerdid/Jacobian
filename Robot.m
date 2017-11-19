%Create SerialLink object with the Denavit-Hartenberg's parameters
robot=SerialLink([Revolute('alpha',-pi/2) Revolute('alpha',pi/2) Revolute('d',3,'alpha',-pi/2) Revolute('alpha',pi/2) Revolute('d',5,'alpha',-pi/2) Revolute('alpha', pi/2) Revolute('d',7)],'name','7DOF Robot');

q=[70,20,-30,60,-15,75,10]; %Angular position (degrees)
qd=[3;2;9;8;5;4;7]; %Angular velocity (rad/s)

%3D plot of the robot at the given joint angles
robot.plot(q,'deg','jointdiam',.2);

% j0=R.jacob0(q,options) is the Jacobian matrix (6 x N) for the robot
% in pose q(1 x N), and N is the number of robot joints. The manipulator
% Jacobian matrix maps joint velocity to the end-effector spatial velocity 
% V=j0*QD expressed in the world-coordinate frame
j0=robot.jacob0(q);

%jdq=R.jacob_dot(q,qd) is the product (6 x 1) of the derivative of the
%Jacobian (in the world frame) and the joint rates.
jdq=robot.jacob_dot(q,qd);

%je=R.jacobe(q,options) is the Jacobian matrix (6 x N) for the robot in 
%pose q, and N is the number of robot joints. The manipulator Jacobian 
%matrix maps joint velocity to end-effector spatial velocity V=je*QD in the
%end-effector frame
je=robot.jacobe(q);

V=je*qd  %Computes spatial velocity