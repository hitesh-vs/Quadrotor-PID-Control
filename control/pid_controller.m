function [F, M, trpy, drpy] = pid_controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

persistent gd;
persistent icnt;
global time_log roll_log roll_des_log pitch_log pitch_des_log yaw_log yaw_des_log;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
     % Initialize logs
     time_log = [];
     roll_log = [];
     roll_des_log = [];
     pitch_log = [];
     pitch_des_log = [];
     yaw_log = [];
     yaw_des_log = [];
 end
 icnt = icnt + 1;
 
% =================== Your code starts here ===================
%% Parameter Initialization



%Hover Position and orientation initialisation
r_des = qd{qn}.pos_des;
rdot_des = qd{qn}.vel_des;
rddot_des = qd{qn}.acc_des;
yaw_des = qd{qn}.yaw_des;
yawdot_des = qd{qn}.yawdot_des;

%Current State info
r = qd{qn}.pos;
rdot = qd{qn}.vel;
roll = qd{qn}.euler(1);
pitch = qd{qn}.euler(2);
yaw = qd{qn}.euler(3);
roll_dot = qd{qn}.omega(1);
pitch_dot = qd{qn}.omega(2);
yaw_dot = qd{qn}.omega(3);

%Parameters Initialisation
m = params.mass;
I = params.I;
%invI = params.invI;
g = params.grav;
%L = params.arm_length;

%Controller Gains
kdz = 550;
kpz = 345;
kpx = 100;
kdx = 10;
kpy = 100;
kdy = 10;
kproll = 650;
kppitch = 650;
kpyaw = 100;
kdroll = 50;
kdpitch = 50;
kdyaw = 100;

%From the equations derived for the hover control
F = m*g - m*(kdz*rdot(3) + kpz*(r(3) - r_des(3)));

r2ddot_des = kdy*rdot(2) + kpy*(r(2) - r_des(2));
r1ddot_des = kdx*rdot(1) + kpx*(r(1) - r_des(1));

roll_des = (r1ddot_des*sin(yaw_des) - r2ddot_des*cos(yaw_des))/g;
pitch_des = (r1ddot_des*cos(yaw_des) + r2ddot_des*sin(yaw_des))/g;

rolldot_des = 0;
pitchdot_des = 0;

%matrix in Attitude control
Matrix = [kproll*(roll_des - roll) + kdroll*(rolldot_des - roll_dot);
          kppitch*(pitch_des - pitch) + kdpitch*(pitchdot_des - pitch_dot);
          kpyaw*(yaw_des - yaw) + kdyaw*(yawdot_des - yaw_dot);]; 

M = I*Matrix;

% Log the data for plotting
time_log = [time_log, t];
roll_log = [roll_log, roll];
roll_des_log = [roll_des_log, roll_des];
pitch_log = [pitch_log, pitch];
pitch_des_log = [pitch_des_log, pitch_des];
yaw_log = [yaw_log, yaw];
yaw_des_log = [yaw_des_log, yaw_des];

% =================== Your code ends here ===================

%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end
