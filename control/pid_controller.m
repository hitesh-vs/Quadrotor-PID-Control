function [F, M, trpy, drpy, logs] = pid_controller(qd, t, qn, params)
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

omega = [roll_dot;pitch_dot;yaw_dot];

%Parameters Initialisation
m = params.mass;
I = params.I;
%invI = params.invI;
g = params.grav;
%L = params.arm_length;

%Controller Gains
kdz = 22;
kpz = 18.5;
kpx = 5;
kdx = 7;
kpy = 9;
kdy = 7;
kproll = 1000;
kppitch = 1000;
kpyaw = 300;
kdroll = 80;
kdpitch = 80;
kdyaw = 62.5;

% errors 
e_p = r_des - r;
e_v = rdot_des - rdot;

%Tracking 
global 	T_pre count P_MSE_list V_MSE_list  P_RMS V_RMS
P_RMS = P_RMS + e_p'*e_p;
V_RMS = V_RMS + e_v'*e_v;
P_MSE_list = [P_MSE_list;e_p'*e_p];
V_MSE_list = [V_MSE_list; e_v'*e_v];
disp("Postion_MSE: "+num2str( e_p'*e_p));
disp("Velocity_MSE: "+num2str(e_v'*e_v));
dt = t - T_pre;
T_pre = t;
count = count + 1;

%Position controller
rddot(1) = rddot_des(1) + kpx*(e_p(1)) + kdx*(e_v(1));
rddot(2) = rddot_des(2) + kpy*(e_p(2)) + kdy*(e_v(2));
rddot(3) = rddot_des(3) + kpz*(e_p(3)) + kdz*(e_v(3));

F = m*(g+rddot(3));

%Attitude Controller
roll_des = 1/g*(rddot(1)*sin(yaw_des)-rddot(2)*cos(yaw_des));
pitch_des = 1/g*(rddot(1)*cos(yaw_des)+rddot(2)*sin(yaw_des));

% global roll_des_pre pitch_des_pre
% rolldot_des = (roll_des - roll_des_pre )/dt;
% pitchdot_des = (pitch_des - pitch_des_pre)/dt;
% roll_des_pre = roll_des;
% pitch_des_pre = pitch_des;

rolldot_des = 0;
pitchdot_des = 0;

Matrix = [kproll*(roll_des - roll) + kdroll*(rolldot_des - roll_dot);
           kppitch*(pitch_des - pitch) + kdpitch*(pitchdot_des - pitch_dot);
           kpyaw*(yaw_des - yaw) + kdyaw*(yawdot_des - yaw_dot);]; 


%disp(rolldot_des)

M = I*Matrix;
% %From the equations derived for the hover control
% F = m*g - m*( kdz*rdot(3) + kpz*(r(3) - r_des(3)));
% 
% r2ddot_des = kdy*rdot(2) + kpy*(r(2) - r_des(2));
% r1ddot_des = kdx*rdot(1) + kpx*(r(1) - r_des(1));
% 
% roll_des = (r1ddot_des*sin(yaw_des) - r2ddot_des*cos(yaw_des))/g;
% pitch_des = (r1ddot_des*cos(yaw_des) + r2ddot_des*sin(yaw_des))/g;
% 
% rolldot_des = 0;
% pitchdot_des = 0;
% 
% %matrix in Attitude control
% Matrix = [kproll*(roll_des - roll) + kdroll*(rolldot_des - roll_dot);
%           kppitch*(pitch_des - pitch) + kdpitch*(pitchdot_des - pitch_dot);
%           kpyaw*(yaw_des - yaw) + kdyaw*(yawdot_des - yaw_dot);]; 
% 
% M = I*Matrix;

% Log the data for plotting
time_log = [time_log, t];
roll_log = [roll_log, roll];
roll_des_log = [roll_des_log, roll_des];
pitch_log = [pitch_log, pitch];
pitch_des_log = [pitch_des_log, pitch_des];
yaw_log = [yaw_log, yaw];
yaw_des_log = [yaw_des_log, yaw_des];

% Store all logs in a single structure for output
logs = struct('time', time_log, ...
              'roll', roll_log, 'roll_des', roll_des_log, ...
              'pitch', pitch_log, 'pitch_des', pitch_des_log, ...
              'yaw', yaw_log, 'yaw_des', yaw_des_log);

% =================== Your code ends here ===================

%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end
