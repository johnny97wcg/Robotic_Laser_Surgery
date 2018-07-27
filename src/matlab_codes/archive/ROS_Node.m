classdef ROS_Node
   properties % class fields
       node
       pub1
       pub2
       pub3
       pub4
       pub5
       pub6
       joint_state_sub
       moveit_goal_sub
       kinematics_obj
       HT
   end
   
   methods % class functions
       function obj = ROS_Node % class constructor
           obj.node = robotics.ros.Node('/matlabKinematics');
           obj.pub1 = robotics.ros.Publisher(obj.node,'/abb_irb120/joint_1_position_controller/command','std_msgs/Float64');
           obj.pub2 = robotics.ros.Publisher(obj.node,'/abb_irb120/joint_2_position_controller/command','std_msgs/Float64');
           obj.pub3 = robotics.ros.Publisher(obj.node,'/abb_irb120/joint_3_position_controller/command','std_msgs/Float64');
           obj.pub4 = robotics.ros.Publisher(obj.node,'/abb_irb120/joint_4_position_controller/command','std_msgs/Float64');
           obj.pub5 = robotics.ros.Publisher(obj.node,'/abb_irb120/joint_5_position_controller/command','std_msgs/Float64');
           obj.pub6 = robotics.ros.Publisher(obj.node,'/abb_irb120/joint_6_position_controller/command','std_msgs/Float64');
           obj.joint_state_sub = robotics.ros.Subscriber(obj.node,'/joint_states','sensor_msgs/JointState'); % no callback function
           obj.moveit_goal_sub = robotics.ros.Subscriber(obj.node,'/moveit_goal','geometry_msgs/Point', @MoveitSubscriberCallback);
           obj.kinematics_obj = Kinematics; % inherit class object from class Kinematics
           obj.HT = load('HT.mat','HT');
       end
       
       function pub_angles(obj,joint_angles)
           % takes in an array of joint angles configurations, and publish them to the according controllers
           msgs = [rosmessage('std_msgs/Float64') rosmessage('std_msgs/Float64') rosmessage('std_msgs/Float64')...
                   rosmessage('std_msgs/Float64') rosmessage('std_msgs/Float64') rosmessage('std_msgs/Float64')];
           for i = 1:6
               msgs(i).Data = joint_angles(i);
               %disp(msgs(i).Data)
               %disp(joint_angles(i));
           end
           send(obj.pub1, msgs(1));
           send(obj.pub2, msgs(2));
           send(obj.pub3, msgs(3));
           send(obj.pub4, msgs(4));
           send(obj.pub5, msgs(5));
           send(obj.pub6, msgs(6));    
       end
       
       function q = get_angles(obj)
           msg = obj.joint_state_sub.LatestMessage;
           q = msg.Position';
       end
              
       function run_trajectory(obj,ps,pf,duration,step)
           dt = duration/step;
           setpoints = obj.kinematics_obj.quintic_trajectory(ps,pf,duration,step); % get setpoints
           target_reached = 0;
           obj.pub_angles(setpoints(1,:)); % publish 1st setpoint
           
           for i = 2:step
               while ~target_reached
                   %disp('trying');
                   pause(dt);
                   current_loc = obj.get_angles;
                   target = setpoints(i-1,:);
                   %disp(norm(current_loc-target));
                   if norm(current_loc -target) < 1e-6
                       target_reached = 1;
                       %disp('reached setpoint');
                   end
               end
               obj.pub_angles(setpoints(i,1:6));
               target_reached = 0;
           end
       end
       
       function p = get_moveit_goal(obj)
           msg = obj.moveit_goal_sub.LatestMessage;
           p = [msg.X, msg.Y, msg.Z];
       end       
        
       function send_moveit_cmd(obj, point)
       % take in a geometry_msgs/Point and publish joint angles
            local_goal = [point.X point.Y point.Z];
            world_goal = [local_goal,0]*obj.HT;
            goal_joint_angles = obj.kinematics_obj.InverseKinematics(world_goal);
            obj.pub_angles(goal_joint_angles);
            disp('goal sent');
       end
       
       function MoveitSubscriberCallback(~, message)
           goal = message.Data;
           obj.send_moveit_cmd(goal);
       end
       
   end
   
end