function values =evaluation_rader_project_module_one(t, sample_time, states, map3d_faces, map3d_struct, terrain, terrain_params)
%EVALUATION_RADER_PROJECT_MODULE_ONE 
% Automatically generated once by read_parameter_xml.m
% This function will be called by evaluation_module_one.m

% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your evaluation submodule, you need to
% get parameters by evaluation_rader_project_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'evaluation_rader_project_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func([file_name_param,str_core]);
end
[v_flock,...
r_coll,...
target] = fun_params();

%
%
number = size(states,2); 
position = states(1:2,:);
velocity = states(4:5,:);

if number > 1
    disMat = pdist(position');
    phiColl = mean(disMat < r_coll);  %#1

else
    phiColl = 0;
    %phiMND = 0;
end
velocity = velocity + rand(size(velocity))*1e-20;
speed = vecnorm(velocity,2,1);
velUnit = velocity./speed;
if number > 1
    phiCorr = (norm(mean(velUnit,2))^2*number - 1)/(number - 1); %#2
else
    phiCorr = norm(mean(velUnit,2))^2;
end
phiVel = mean(speed)/v_flock;                                    %#3

out_of_map = map_module_out_of_map(states(1:3,:),map3d_struct);
col_map = map_module_collision_detection(states(1:3,:),map3d_faces,r_coll/2);
outOfMap = out_of_map | col_map;

phiWall = sum(outOfMap)/number;
phigroup = group_number(position')/number;
values = [phiCorr;phiVel;phiColl;phiWall;phigroup];

%success_rate = strcat(num2str(success_define(states,r_coll, map3d_faces, map3d_struct)),'%');

success_rate = (success_define(states,r_coll, map3d_faces, map3d_struct,target));




end