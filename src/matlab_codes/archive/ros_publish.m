function handle = ros_publish %code interation ONE
    % sutup publishers
    [joint1_pub, msg1] = rospublisher('/abb_irb120/joint_1_position_controller/command','std_msgs/Float64');
    [joint2_pub, msg2] = rospublisher('/abb_irb120/joint_2_position_controller/command','std_msgs/Float64');
    [joint3_pub, msg3] = rospublisher('/abb_irb120/joint_3_position_controller/command','std_msgs/Float64');
    [joint4_pub, msg4] = rospublisher('/abb_irb120/joint_4_position_controller/command','std_msgs/Float64');
    [joint5_pub, msg5] = rospublisher('/abb_irb120/joint_5_position_controller/command','std_msgs/Float64');
    [joint6_pub, msg6] = rospublisher('/abb_irb120/joint_6_position_controller/command','std_msgs/Float64');
    disp('setup publisher');
    
    handle = @pub_joint_config;
    % parent function initializes ros and publisher, only called once
    % child function publishes joint angle, called with handle
    
    function pub_joint_config(q)
    % takes in an array of joint angles configurations, and publish them to the according controllers
        msg1.Data = q(1);
        msg2.Data = q(2);
        msg3.Data = q(3);
        msg4.Data = q(4);
        msg5.Data = q(5);
        msg6.Data = q(6);
        send(joint1_pub, msg1);
        send(joint2_pub, msg2);
        send(joint3_pub, msg3);
        send(joint4_pub, msg4);
        send(joint5_pub, msg5);
        send(joint6_pub, msg6);
    end
end