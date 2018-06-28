function generate_trajectory(ps, pf) %take in xyz vector of start and end location 

    global ik_handle;
    global pub_handle;
    duration = 5; % duration of trajectory in seconds
    interval = 100;
    dt = duration/interval; 
    
    
    if ps == 0 % when one of the inputs is set to 0, the arm departs from or returns to home position
        joint_ang_start = zeros(1,6);
    else
        joint_ang_start = ik_handle(ps);
    end
    %disp(joint_ang_start);
    if pf == 0
        joint_ang_end = zeros(1,6);
    else
        joint_ang_end = ik_handle(pf);
    end
    %disp(joint_ang_end);

    
    coefficients = zeros(6);
    for i = 1:6
        coefficients(i,:) = quinticPolynomial(0,duration,joint_ang_start(i), joint_ang_end(i),0,0,0,0);
        % 6x6 matrix of quintic coefficients, each row corresponds to each joint
    end
    %disp(coefficients);

    
    joint_vals = zeros(6,interval); % matrix containing joint values of each incremental movement
    % rows[i] represent each joint, columns[j] represents time instance dt
    for i = 1:6
        for j = 1:interval
            joint_vals(i,j) = get_instant_angle(coefficients(i,:),dt*(j-1));
        end
    end
    %disp(joint_vals(:,interval));
    
    
    disp('start publishing');
    for j = 1:interval
        pub_handle(joint_vals(:,j));
        pause(dt);
    end
end



