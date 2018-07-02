% created by Chenggu Wang on 6/27/18
% this is a script that runs the Iterative Closest Point registeration
% between robot reference frame and workobject reference frame,
% and thus localize the workobject in the robot's coordinate system.

% all units in mm for position and rad for orientation

% local xyz positions of known calibration points on the workobject
% origin is the top right corner
workobj_p1 = [65 5 0]';
workobj_p2 = [35 35 0]';
workobj_p3 = [95 35 0]';
workobj_p4 = [65 65 0]';
workobj_p5 = [5 95 0]';
workobj_p6 = [125 95 0]';
workobj_X = [workobj_p1, workobj_p2, workobj_p3, workobj_p4, workobj_p5, workobj_p6];

% move robot to corresponding calibration points and record joint angles
robot_q1 = [1.54879 0.88788 0.49963 0.01240 0.20336 0.21891];
robot_q2 = [1.63390 0.93609 0.37036 0.01494 0.28426 0.30108];
robot_q3 = [1.46855 0.93930 0.36315 0.00338 0.28800 0.14688];
robot_q4 = [1.55182 0.99284 0.23608 0.00738 0.36137 0.22653];
robot_q5 = [1.69601 1.06163 0.07636 0.01234 0.45179 0.36660];
robot_q6 = [1.41435 1.06829 0.06105 0.00000 0.46124 0.09612];

% create a kinematics class object
kin_obj = Kinematics;

% convert robot joint angles to location in robot's reference frame
robot_p1 = kin_obj.ForwardKinematics(robot_q1);
robot_p2 = kin_obj.ForwardKinematics(robot_q2);
robot_p3 = kin_obj.ForwardKinematics(robot_q3);
robot_p4 = kin_obj.ForwardKinematics(robot_q4);
robot_p5 = kin_obj.ForwardKinematics(robot_q5);
robot_p6 = kin_obj.ForwardKinematics(robot_q6);
robot_Y = 1000*[robot_p1, robot_p2, robot_p3, robot_p4, robot_p5, robot_p6];

% run the ICP point registeration script
[R,t,FRE,FREcomponents] = point_register(workobj_X,robot_Y);
% construct homogeneous transformation matrix
HT = [R,t;[0 0 0 1]];
% save point registeration data to a mat file for furture use
save('HT.mat', 'HT');
disp('writing transformation matrix to file');

