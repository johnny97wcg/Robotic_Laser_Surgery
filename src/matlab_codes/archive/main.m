%this is the main file
% rosinit; %initialize ros master
setenv('ROS_MASTER_URI','http://halley:11311')
setenv('ROS_HOSTNAME','halley')

p_start = [300 0 0];
p_end = [300 200 300];

node = ROS_Node;
node.run_trajectory(0,p_start,2,20);
node.run_trajectory(p_start,p_end,2,20);
node.run_trajectory(p_end,0,2,20);

pause(10);
disp('shutting down rosmaster');
%rosshutdown;
clearvars;