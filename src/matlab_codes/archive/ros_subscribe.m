function handle = ros_subscribe
    % setup subscribers
    joint_states_sub = rossubscriber('/joint_states','sensor_msgs/JointState',@joint_states_callback);
    
    function joint_states_callback(~,msg)
        global joint_angles;
        joint_angles = msg.Position;
    end
    
    function q = get_joint_angles
        msg = joint_states_sub.LatestMessage;
        q = msg.Position;
    end

    handle = @get_joint_angles;
end