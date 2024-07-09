function [command_upper_s,control_mode_s] =pf_i_module_generate_desire(t, states, sample_time, sensor_data_s, map3d_struct, terrain, terrain_params, values)
%PF_I_MODULE_GENERATE_DESIRE Generate the desired position and velocity
% Automatically generated once by read_parameter_xml.m
% This function will be called by swarms_module_generate_desire.m
%   point-mass: state = [x; y; z; vx; vy; vz; ax; ay; az]
%   quadcopter: state = [x; y; z; vx; vy; vz; ax; ay; az; yaw; roll; Pitch];
% control_mode:
% TAKEOFF_TYPE = 2;
% HOVER_TYPE = 3;
% LAND_TYPE = 4;
% POSITION_CONTROL_TYPE = 5;
% VELOCITY_CONTROL_TYPE = 6;
% VELOCITY_HORIZONTAL_CONTROL_TYPE = 7;


% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your swarm submodule, you need to
% get parameters by pf_i_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'pf_i_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func([file_name_param,str_core]);
end

[ijR,...
cR,...
obR,...
alpha_attij,...
alpha_rejij,...
alpha_rejob,...
alpha_atttar,...
v_max,...
dim,...
height,...
dr_shill,...
rader_boold_min,...
pos_shill,...
vel_shill] = fun_params();

%
number = size(states,2);
command_upper_s = zeros(12,number);
command_upper_s(1:3,:) = states(1:3,:);
control_mode_s = uint8(zeros(1,number))+6;

% canyon_snow_map
if states(1,1) < -140
    target = [-70;-11;120];
else
    target=[300;-10;180];
end

% Dead_end_canyon_map
% target=[0;-280;30];



% obs1 = [pos_shill;height.*ones(1,size(pos_shill,2))];
% obs2 = [pos_shill;(height+8).*ones(1,size(pos_shill,2))];
% obs3 = [pos_shill;(height-8).*ones(1,size(pos_shill,2))];
% obs4 = [pos_shill;(height+16).*ones(1,size(pos_shill,2))];
% obs5 = [pos_shill;(height-16).*ones(1,size(pos_shill,2))];
% obs6 = [pos_shill;(height+24).*ones(1,size(pos_shill,2))];
% obs7 = [pos_shill;(height-24).*ones(1,size(pos_shill,2))];
% obs8 = [pos_shill;(height+32).*ones(1,size(pos_shill,2))];
% obs9 = [pos_shill;(height-32).*ones(1,size(pos_shill,2))];
% obs10 = [pos_shill;(height+40).*ones(1,size(pos_shill,2))];
% obs11 = [pos_shill;(height-40).*ones(1,size(pos_shill,2))];
% obs = [obs1 obs2 obs3 obs4 obs5 obs6 obs7 obs8 obs9 obs10 obs11];


% Traverse every agent
for id  = 1:number
	pos_desired_id = [states(1:3,id);0];
    position=[states(1:3,:)];

	acc_desired_id = [0;0;0;0];
    
    state_i = states(:,id);
    [obs,vel_shill] = map_module_generate_shill_agent_by_terrain(state_i,terrain,terrain_params,obR);

    [vel_desired_id] = pf_i(id,number,position,target,obs,ijR,cR,obR,alpha_attij,alpha_rejij,alpha_rejob,alpha_atttar,v_max);
    agent_blood = values(id);
    if agent_blood < rader_boold_min
        if position(3,id) > 0 
            vel_desired_id = [0;0;-5];
        else
            vel_desired_id = [0;0;0];
        end
    end


    %%
	command_upper_s(:,id) = [pos_desired_id;[vel_desired_id(1:3);0];acc_desired_id];

end

end