%% Kinematics Server

% launch ros node
node = robotics.ros.Node('/matlabKinematics');

% define publishers
global pub1 pub2 pub3 pub4 pub5 pub6
pub1 = robotics.ros.Publisher(node,'/abb_irb120/joint_1_position_controller/command','std_msgs/Float64');
pub2 = robotics.ros.Publisher(node,'/abb_irb120/joint_2_position_controller/command','std_msgs/Float64');
pub3 = robotics.ros.Publisher(node,'/abb_irb120/joint_3_position_controller/command','std_msgs/Float64');
pub4 = robotics.ros.Publisher(node,'/abb_irb120/joint_4_position_controller/command','std_msgs/Float64');
pub5 = robotics.ros.Publisher(node,'/abb_irb120/joint_5_position_controller/command','std_msgs/Float64');
pub6 = robotics.ros.Publisher(node,'/abb_irb120/joint_6_position_controller/command','std_msgs/Float64');

% define subscribers
joint_state_sub = robotics.ros.Subscriber(node,'/joint_states','sensor_msgs/JointState'); % no callback function
moveit_goal_sub = robotics.ros.Subscriber(node,'/moveit_goal','geometry_msgs/Point', @MoveitSubscriberCallback);

% create a Kinematics object
global kin
kin = Kinematics();

% load the Registration matrix
global HT
HT = load('HT.mat','HT');


function pub_angles(joint_angles)
    global pub1 pub2 pub3 pub4 pub5 pub6
    
    % takes in an array of joint angles configurations, and publish them to the according controllers
    msgs = [rosmessage('std_msgs/Float64') rosmessage('std_msgs/Float64') rosmessage('std_msgs/Float64')...
        rosmessage('std_msgs/Float64') rosmessage('std_msgs/Float64') rosmessage('std_msgs/Float64')];

    for i = 1:6
        msgs(i).Data = joint_angles(i);
        %disp(msgs(i).Data)
        %disp(joint_angles(i));
    end
    
    send(pub1, msgs(1));
    send(pub2, msgs(2));
    send(pub3, msgs(3));
    send(pub4, msgs(4));
    send(pub5, msgs(5));
    send(pub6, msgs(6));
end


function send_moveit_cmd(point)
    global kin
    global HT
    % take in a geometry_msgs/Point and publish joint angles
    local_goal = point;
    world_goal = HT.HT * [local_goal; 1];
    goal_joint_angles = kin.InverseKinematics(world_goal);
    pub_angles(goal_joint_angles);
    disp('goal sent');
end

function MoveitSubscriberCallback(~, message)
    goal = [message.X; message.Y; message.Z];
    send_moveit_cmd(goal);
end



