function [gravity,...
inertia,...
mass,...
len_arm,...
v_max_h,...
v_max_v,...
yaw_rate_max,...
a_max_h,...
a_max_v,...
euler_max,...
thrust_max,...
ct,...
cm,...
T_rotor_inverse,...
kp_att,...
kd_att,...
kp_pos,...
ki_pos,...
kd_pos,...
cd_filter_pos,...
lb_pos_pid,...
ub_pos_pid,...
kp_vel,...
ki_vel,...
kd_vel,...
cd_filter_vel,...
lb_vel_pid,...
ub_vel_pid] = quadcopter_module_parameters_7()
%QUADCOPTER_MODULE_PARAMETERS_7 
% Automatically generated by read_parameter_xml.m
% Every time read_parameter_xml.m is run, this function will be generated
gravity = 9.810000000000;
inertia = [0.000290833000
0.000290833000
0.000540000000];
mass = 0.092000000000;
len_arm = 0.060000000000;
v_max_h = 1.500000000000;
v_max_v = 1.000000000000;
yaw_rate_max = 0.500000000000;
a_max_h = 2.000000000000;
a_max_v = 2.000000000000;
euler_max = 0.261799387799;
thrust_max = 2.256300000000;
ct = 0.000000122200;
cm = 0.000000003000;
T_rotor_inverse = 20.000000000000;
kp_att = [0.087250000000
0.087250000000
0.162000000000];
kd_att = [0.017500000000
0.017500000000
0.032400000000];
kp_pos = [1.000000000000
1.000000000000
1.000000000000
0.100000000000];
ki_pos = [0.000000000000
0.000000000000
0.000000000000
0.000000000000];
kd_pos = [0.000000000000
0.000000000000
0.000000000000
0.000000000000];
cd_filter_pos = 0.400000000000;
lb_pos_pid = [-2.000000000000
-2.000000000000
-2.000000000000
-2.000000000000];
ub_pos_pid = [2.000000000000
2.000000000000
2.000000000000
2.000000000000];
kp_vel = [2.200000000000
2.200000000000
1.000000000000];
ki_vel = [0.000000000000
0.000000000000
0.000000000000];
kd_vel = [0.000000000000
0.000000000000
0.000000000000];
cd_filter_vel = 0.400000000000;
lb_vel_pid = [-2.000000000000
-2.000000000000
-2.000000000000];
ub_vel_pid = [2.000000000000
2.000000000000
2.000000000000];


end