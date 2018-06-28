classdef Kinematics
   properties % class fields
       robot
       ik_solver
   end
   
   methods % class functions
       function obj = Kinematics % class constructor
           % create robot model from urdf file
           obj.robot = importrobot('/home/halley/sandbox/src/irb120_description/urdf/abb_irb120.urdf');
           obj.ik_solver = robotics.InverseKinematics('RigidBodyTree',obj.robot);
       end
       
       function q = InverseKinematics(obj,xyz)
           % Define a random location and orientation and solve the inverse kinematics, xyz has unit in mm
           P = 1e-3 * [xyz(1) xyz(2) xyz(3)]'; % [target location in meters] 
           
           % Calculate the rotation matrix using Rodrigues formula
           K = zeros(3,3); K(1,3) = 1; K(3,1) = -1; 
           R = eye(3) + K + K^2;
           HT = [R P; 0 0 0 1]; % homogeneous transformation matrix
           
           [targetConf, ~] = obj.ik_solver('end_effector', HT, 1e-6 * ones(1,6), obj.robot.homeConfiguration);
           
           q =  zeros(1,6);
           for i = 1:6
               q(i) = targetConf(i).JointPosition;
           end
           
           % code for bringing up robot model in matlab
           show(obj.robot, targetConf);


       end
        
       function p = ForwardKinematics(obj,q)
           targetConf = obj.robot.homeConfiguration(); % initialize configuration
           for i = 1:6
               targetConf(i).JointPosition = q(i); % parse joint values q to config
           end
           tform = obj.robot.getTransform(targetConf, 'end_effector', 'world');
           p = tform(1:3,4);
           show(obj.robot, targetConf);
       end
       
       function set_points = quintic_trajectory(obj,ps,pf,duration,step)
            %take in xyz vector of start and end location, total duration of the path in seconds, and number of intervals
            dt = duration/step;
            
            if ps == 0 % when one of the inputs is set to 0, the arm departs from or returns to home position
                joint_ang_start = zeros(1,6);
            else
                joint_ang_start = obj.InverseKinematics(ps);
            end
            if pf == 0
                joint_ang_end = zeros(1,6);
            else    
                joint_ang_end = obj.InverseKinematics(pf);
            end
            
            coefficients = zeros(6);
            for i = 1:6
                coefficients(i,:) = quinticPolynomial(0,duration,joint_ang_start(i), joint_ang_end(i),0,0,0,0);
                % 6x6 matrix of quintic coefficients, each row corresponds to each joint
            end
            
            set_points = zeros(step,6); 
            % matrix containing joint values of each incremental movement 
            for i = 1:step
                for j = 1:6
                    set_points(i,j) = get_instant_angle(coefficients(j,:),dt*i);
                end
            end
        end
     
   end
   
end
