function handle = ik_irb120
    % Location of URDF file
    urdf_path = '/home/halley/catkin_ws/src/abb_irb120/irb120_description/urdf/abb_irb120.urdf';

    % Create robot object
    robot = importrobot(urdf_path);

    % Create Inverse Kinematics Solver
    ik = robotics.InverseKinematics('RigidBodyTree', robot);

    handle = @get_config;
    % handle calls on the nested function to calculate joint angles. 
    % the parent function define robot object and ik solver, the child function does the calculation 
    % usage: call parent function once to initialize handle, and use the handle to call child function 

    function q = get_config(xyz)

        % Define a random location and orientation and solve the inverse kinematics
        P = 1e-3 * [xyz(1) xyz(2) xyz(3)]'; % [target location in meters] [home config: 350 0 300]

        % Calculate the rotation matrix using Rodrigues formula
        K = zeros(3,3); K(1,3) = 1; K(3,1) = -1; 
        R = eye(3) + K + K^2;

        HT = [R P; 0 0 0 1]; % homogeneous transformation matrix

        [targetConf, solInfo] = ik('end_effector', HT, 1e-6 * ones(1,6), robot.homeConfiguration);
        q =  zeros(1,6);
        for i = 1:6
            q(i) = targetConf(i).JointPosition;
        end
        
        % code for bringing up robot model in matlab and displaying end_effector location
%         show(robot, targetConf);
%         tform = getTransform(robot, targetConf, 'end_effector', 'base_link');
%         disp('target location:');
%         xyz = tform(1:3,3)';
%         disp(xyz);
%         %tip = robot.getBody('end_effector'); % manually add visual for the end-effector
%         %tip.addVisual('Mesh','/home/halley/catkin_ws/src/abb_irb120/irb120_description/meshes/visual/new.STL', tform)

    end
end


