robot=SerialLink([Revolute('alpha',-pi/2) Revolute('alpha',pi/2) Revolute('d',3,'alpha',-pi/2) Revolute('alpha',pi/2) Revolute('d',5,'alpha',-pi/2) Revolute('alpha', pi/2) Revolute('d',7)],'name','7DOF Robot');

robot.plot([70,20,-30,60,-15,75,10],'deg','jointdiam',.2);
