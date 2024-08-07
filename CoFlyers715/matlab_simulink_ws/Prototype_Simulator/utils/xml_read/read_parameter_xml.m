function [map3d_faces, map3d_struct, model_stls, params,...
    position0, param_simulink,...
    terrain, terrain_params] = read_parameter_xml(file_name_xml, ps_mofify, flag_write)
%READ_PARAMETER_XML Obtain parameters for each module from an XML file
%   file_name_xml: path of the XML file relative to the path of Prototype_Simulator
%   ps_mofify: external input parameters that will modify default parameters
%   flag_write: flag for deciding whether to write new files for modules

%%% Obtain the current CPU core serial number %%%
[flag_parallel,str_core] = get_multi_core_value();

%%% Check the input arguments %%%
if nargin < 1
    file_name_xml = "xml_config_files\parameters.xml";
    ps_mofify = struct("param_name_s",[],"param_value_s",[]);
    flag_write = true;
elseif nargin < 2
    ps_mofify = struct("param_name_s",[],"param_value_s",[]);
    flag_write = true;
elseif nargin < 3
    flag_write = true;
end
if isempty(ps_mofify)
    ps_mofify = struct("param_name_s",[],"param_value_s",[]);
end
if isempty(file_name_xml)
    file_name_xml = "parameters.xml";
end
if isempty(file_name_xml)
    file_name_xml = "parameters.xml";
end
%% Get params from XML and ps_mofify

%%% Translate XML to Struct %%%
first_name = "CoFlyers";
param_xml = parseXML(file_name_xml);
[CoFlyers, param_string] = parseXML2(param_xml, first_name);
init_map = false;

%%% Get the creation time of the xml file %%%
xml_file = dir(file_name_xml);
ctime_xml = datenum(xml_file.date);


%%% Modify the default parameters 'CoFlyers' and 'param_string' according to
% 'ps_mofify' %%%
for ii = 1:length(ps_mofify.param_value_s)
    param_value = ps_mofify.param_value_s(ii);
    param_name = ps_mofify.param_name_s(ii);
    temp = str2double(param_value);
    if ~isnan(temp)
        param_value = temp;
    else
        param_value = char(param_value);
    end
    try
        % Determine whether the nested field exists.
        eval(strcat(param_name,";"));
        % Set value
        if ischar(param_value)
            str_eval = strcat(param_name,"=",param_value,";");
        else
            str_eval = strcat(param_name,"=",num2str(param_value),";");
        end
        eval(str_eval);
        % Remove ref
        ind_1 = strcmp(param_string(2,:),param_name);
        param_string(:,ind_1) = [];
    catch
        error(strcat("The name of parameter ",param_name," is wrong."))
    end
end

%%% Find models of the map module %%%
temp_11 = strfind(param_string(2,:),strcat(first_name,".map.model"));
temp_22 = strfind(param_string(2,:),".stl");
is_model = arrayfun(@(x)~isempty(temp_11{x})&&~isempty(temp_22{x}),1:length(temp_11));
temp_11 = strfind(param_string(2,:),strcat(first_name,".map.terrain."));
is_terrain = arrayfun(@(x)~isempty(temp_11{x}),1:length(temp_11));
if sum(is_terrain) > 1
    error("The number of terrain models can only be less than or equal to 1.")
end
num_model = sum(is_model)+sum(is_terrain);


%%% Deal with MATLAB commands with ref %%%
count = 1;
ind_delete = [];
while size(param_string,2)>num_model
    for s = 1:size(param_string,2)
        if is_model(s)
            continue
        end
        field_string = param_string(2,s);
        field_value = param_string(1,s);
        if contains(field_value,param_string(2,:))
            continue
        end
        %%% deal with multiple ouput of function %%%
        temp1 = strsplit(field_string,".");
        temp2 = strsplit(temp1{end},"-");
        field_string_s = cellfun(@(x)strjoin([temp1(1:end-1),x],"."),temp2);
        %%% run MATLAB Commands %%%
        try
            str_eval = strcat("[",field_string_s(1),...
                arrayfun(@(x)strcat(",",x),field_string_s(2:end)),"] = ", field_value,";");
            str_eval = strrep(str_eval,"...","");
            eval(str_eval);
            ind_delete = [ind_delete,s];
        catch

        end
    end
    param_string(:,ind_delete) = [];
    ind_delete = [];

    %%%
    count = count +1;
    if count>100
        str_error = "";
        for ii = 1:length(param_string)
            str_error = strcat(str_error,"Cannot deal with the MATLAB command of ",param_string(1,ii)," in ",param_string(2,ii),".");
            if ii ~=length(param_string)
                str_error = strcat(str_error,"\n");
            end
        end
        %         error("xml error");
        error(strcat("%s file error.\n",str_error),file_name_xml);
    end

    %%%  Get map %%%
    temp = contains(param_string(2,:),".map.")|...
        contains(param_string(1,:),".map.")|...
        contains(param_string(1,:),"map3d_faces")|...
        contains(param_string(1,:),"map3d_struct");
    if ~init_map && (sum(temp)==numel(temp)||size(param_string,2)==num_model)
        % Generate map
        [map3d_faces, map3d_struct, model_stls,...
            terrain, terrain_params,...
            ind_models, ind_terrain, init_map] = get_map(CoFlyers);
    end
end
position0 = CoFlyers.position__;
% CoFlyers = rmfield(CoFlyers, "position__");
param_simulink = CoFlyers.simulink;
% CoFlyers = rmfield(CoFlyers, "simulink");

%%% Remove XXX__ params %%%
CoFlyers = remove_XXX__(CoFlyers);
params = CoFlyers;
%% Check params
head = "XML parameter file error: ";
if size(position0,1)~=3 || size(position0,2) ~= CoFlyers.number
    error(strcat(head, "the dimension of 'position__' must be exactly 3×number."));
end
if ~isfield(CoFlyers.evaluation,CoFlyers.evaluation_metric_type)
    error(strcat(head, "unknown type '%s'.",CoFlyers.evaluation_metric_type));
end
if ~isfield(CoFlyers.swarm,CoFlyers.swarm_algorithm_type)
    error(strcat(head, "unknown type '%s'.",CoFlyers.swarm_algorithm_type));
end
if ~isfield(CoFlyers.motion,CoFlyers.motion_model_type)
    error(strcat(head, "unknown type '%s'.",CoFlyers.motion_model_type));
end

%% Write params to file
%%%
if ~flag_write
    return
end

%%% Get the path of the prototype simulator %%%
path_simulator = mfilename('fullpath');
ind = strfind(path_simulator,'Prototype_Simulator') +...
    length('Prototype_Simulator');
path_simulator = path_simulator(1:ind);

%%% Get setting values %%%
[values, values_name, modules_name] = get_values_and_names(params);

%%% write setting parameter %%%
dir_name_1 = strcat(path_simulator,"combine_modules");
if ~exist(dir_name_1,'dir')
    mkdir(dir_name_1);
end
file_name = strcat("setting_parameters",str_core);
write_params_for_file(dir_name_1, file_name, values, values_name, flag_parallel);

clear setting_parameters.m

%%% write parameters of all modules
for ii = 1:length(modules_name)
    module_name = modules_name(ii);
    %%% Params of simulink do not need to be written in a file. %%%
    if strcmp(module_name,"simulink")
        break;
    end
    %%%
    [values, values_name, submodules_name] = get_values_and_names(params.(module_name));
    dir_name_1 = strcat(path_simulator,"lower_layers");
    if ~exist(dir_name_1,'dir')
        mkdir(dir_name_1);
    end
    dir_name_2 = strcat(dir_name_1,"/", module_name, "_module");
    if ~exist(dir_name_2,'dir')
        mkdir(dir_name_2);
    end
    file_name = strcat(module_name,"_module_parameters",str_core);
    write_params_for_file(dir_name_2, file_name, values, values_name, flag_parallel);
    clear(strcat(file_name,".m"))

    %%% has submodules
    if ~isempty(submodules_name)
        for jj = 1:length(submodules_name)
            submodule_name = submodules_name(jj);
            %%% Params of the terrain and models do not need to be written in a file. %%%
            if strcmp(module_name,"map") && (strcmp(submodule_name,"terrain") || contains(submodule_name,"model"))
                break;
            end
            %%%
            [values, values_name] = get_values_and_names(params.(module_name).(submodule_name));
            dir_name_3 = strcat(dir_name_2,"/", submodule_name, "_module");
            if ~exist(dir_name_3,'dir')
                mkdir(dir_name_3);
            end
            file_name = strcat(submodule_name,"_module_parameters",str_core);
            write_params_for_file(dir_name_3, file_name, values, values_name, flag_parallel);
            clear(strcat(file_name,".m"))
        end
    end
end

%% Write swarm module
%%% Write swarm_module_generate_desire.m %%%
dir_name_sm = strcat(path_simulator,"lower_layers/swarm_module");
file_name_sm = "swarm_module_generate_desire";
field_names_swarm  = fieldnames(CoFlyers.swarm);
ind_subswarm = cellfun(@(x)isstruct(CoFlyers.swarm.(x)),field_names_swarm);
field_names_swarm = field_names_swarm(ind_subswarm);
write_swarm_module_generate_desire(dir_name_sm, file_name_sm, field_names_swarm, ctime_xml);
clear(strcat(file_name_sm,".m"));
%%% Write each function of subswarm model %%%
for ii = 1:length(field_names_swarm)
    dir_name_ssm = strcat(path_simulator,"lower_layers/swarm_module/",...
        field_names_swarm{ii},"_module");
    file_name_ssm = strcat(field_names_swarm{ii},"_module_generate_desire");
    [values, values_name] = get_values_and_names(CoFlyers.('swarm').(field_names_swarm{ii}));
    write_subswarm_for_file(dir_name_ssm, file_name_ssm, field_names_swarm{ii}, values, values_name);
    clear(strcat(file_name_ssm,".m"));
end
%% Write evaluation module
%%% Write evaluation_module_one.m %%%
dir_name_em = strcat(path_simulator,"lower_layers/evaluation_module");
file_name_em = "evaluation_module_one";
field_names_eva  = fieldnames(CoFlyers.evaluation);
ind_subeva = cellfun(@(x)isstruct(CoFlyers.evaluation.(x)),field_names_eva);
field_names_eva = field_names_eva(ind_subeva);
write_evaluation_module_one(dir_name_em, file_name_em, field_names_eva, ctime_xml);
clear(strcat(file_name_em,".m"));
%%% %%%
dir_name_em = strcat(path_simulator,"lower_layers/evaluation_module");
file_name_em = "evaluation_module_average";
field_names_eva  = fieldnames(CoFlyers.evaluation);
ind_subeva = cellfun(@(x)isstruct(CoFlyers.evaluation.(x)),field_names_eva);
field_names_eva = field_names_eva(ind_subeva);
write_evaluation_module_average(dir_name_em, file_name_em, field_names_eva, ctime_xml);

% %%% Write each function of subevaluation model %%%
for ii = 1:length(field_names_eva)
    dir_name_sem = strcat(path_simulator,"lower_layers/evaluation_module/",...
        field_names_eva{ii},"_module");
    file_name_sem = strcat(field_names_eva{ii},"_module_one");
    [values, values_name] = get_values_and_names(CoFlyers.('evaluation').(field_names_eva{ii}));
    write_subevaluation_one_for_file(dir_name_sem, file_name_sem, field_names_eva{ii}, values, values_name);
    clear(strcat(file_name_sem,".m"));
    file_name_sem = strcat(field_names_eva{ii},"_module_average");
    [values, values_name] = get_values_and_names(CoFlyers.('evaluation').(field_names_eva{ii}));
    write_subevaluation_average_for_file(dir_name_sem, file_name_sem, field_names_eva{ii}, values, values_name);
    clear(strcat(file_name_sem,".m"));
end


%%

addpath(genpath(path_simulator));

%%
%%% Functions %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function [map3d_faces, map3d_struct, model_stls,...
        terrain, terrain_params,...    
        ind_models, ind_terrain, init_map] = get_map(params)
        % Generate map
        [map3d_struct_0, model_stls, ind_models, ind_terrain] = read_map_param_struct(params.map);
        
        [map3d_faces, map3d_struct] = generate_map3d_from_struct(map3d_struct_0, model_stls);
        init_map = true;
        % Get terrain
        if ~isempty(ind_terrain)
            [terrain, terrain_params] = get_terrain(params.map.terrain.png, params.map.terrain.position, params.map.terrain.scale);
        else
            terrain = [];
            terrain_params = [];
        end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    function result = get_struct_string(param_struct_sub, field_name_pre)
        result = string([]);
        names = fieldnames(param_struct_sub);
        for i = 1:length(names)
            submodule_param = param_struct_sub.(names{i});
            if ~isstruct(submodule_param)
                if ~isnumeric(submodule_param)
                    if isempty(field_name_pre)
                        str = names{i};
                    else
                        str = strcat(field_name_pre,'.',names{i});
                    end
                    result = [result,[string(submodule_param);str]]; % Get string
                end
            else
                if isempty(field_name_pre)
                    str = names{i};
                else
                    str = strcat(field_name_pre,'.',names{i});
                end
                result = [result, get_struct_string(submodule_param,str)];
            end
        end
    end


    function [values, values_name, subfields_name] = get_values_and_names(params)
        values = {};
        values_name = [];
        subfields_name = [];
        field_names = fieldnames(params);
        for i = 1:length(field_names)
            field_name = field_names{i};
            field_data = params.(field_name);
            if ~isstruct(field_data) %&& ~isempty(field_data)
                %                 if isnumeric(field_data) && ~isempty(field_data)
                values_name = [string(values_name), field_name];
                values = [values, {field_data}];
                %                 end
            else
                subfields_name = [subfields_name, string(field_name)];
            end
        end
    end

    function params_struct = remove_XXX__(params_struct)
        field_names = fieldnames(params_struct);
        temp_1 = strfind(field_names,"__");
        temp_2 = find(cellfun(@(x)~isempty(x),temp_1));
        temp_3 = arrayfun(@(x)strlength(field_names{x}),temp_2);
        temp_4 = arrayfun(@(x)temp_1{x}(end),temp_2);
        ind_delete_5 = temp_2(find((temp_3 - 1) == temp_4));
        for i = 1:length(ind_delete_5)
            params_struct = rmfield(params_struct,field_names{ind_delete_5(i)});
        end
        field_names = fieldnames(params_struct);
        for i = 1:length(field_names)
            if isstruct(params_struct.(field_names{i}))
                params_struct.(field_names{i}) = remove_XXX__(params_struct.(field_names{i}));
            end
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function write_params_for_file(dir_name, file_name, values, values_name, flag_parallel)
        if flag_parallel
            dir_name = strcat(dir_name,"/","params_for_parallel");
            if ~exist(dir_name,'dir')
                mkdir(dir_name);
            end
        end
        file_fullpath = strcat(dir_name,"/", file_name,".m");

        %%% write function name %%%
        file_id = fopen(file_fullpath,'w');
        fprintf(file_id,'function [');
        for i = 1:length(values_name)
            fprintf(file_id,'%s', values_name(i));
            if i ~= length(values_name)
                fprintf(file_id,',...\n');
            end
        end
        fprintf(file_id,'] = %s()\n',file_name);
        

        fprintf(file_id,['%%%s \n',...
            '%% Automatically generated by read_parameter_xml.m\n',...
            '%% Every time read_parameter_xml.m is run, this function will be generated\n'],upper(file_name));

        %%% write values %%%
        for i = 1:length(values_name)

            value_now = values{i};
            if numel(value_now) == 1
                fprintf(file_id,'%s = ',values_name(i));
            else
                fprintf(file_id,'%s = [',values_name(i));
            end
            for j = 1:size(value_now,1)
                for k = 1:size(value_now,2)
                    if isnumeric(value_now(j,k))
                        fprintf(file_id,'%.12f',value_now(j,k));  %%%%%%%%%%%%%%%% 12
                    elseif ischar(value_now(j,k))
                        fprintf(file_id,'''%c''',value_now(j,k));
                    elseif isstring(value_now(j,k))
                        fprintf(file_id,'"%s"',value_now(j,k));
                    end
%                     fwrite(file_id,value_now(j,k));
                    if k ~=size(value_now,2)
                        fprintf(file_id,', ');
                    end
                end
                if j ~=size(value_now,1)
                    fprintf(file_id,'\n');
                end
            end

            if numel(value_now) == 1
                fprintf(file_id,';\n');
            else
                fprintf(file_id,'];\n');
            end
        end

        %%% write end%%%
        fprintf(file_id,'\n\nend');
        fclose(file_id);
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function write_swarm_module_generate_desire(dir_name, file_name, field_names, ctime_xml)
        file_fullpath = strcat(dir_name,"/",file_name,".m");
        % if exist(file_fullpath,"file")
        %     file_now = dir(file_fullpath);
        %     ctime_file = datenum(file_now.date);
        %     if ctime_file > ctime_xml
        %         return
        %     end
        % end
        %%% write function name %%%
        file_id = fopen(file_fullpath,'w');
        fprintf(file_id,['function [command_upper_s,control_mode_s] = swarm_module_generate_desire(swarm_algorithm_type, t, states, sample_time, sensor_data_s, map3d_struct, terrain, terrain_params, values)\n']);

        fprintf(file_id,['%%SWARM_MODULE_GENERATE_DESIRE Generate the desired position and velocity\n',...
            '%% Automatically generated by read_parameter_xml.m\n',...
            '%% Every time read_parameter_xml.m is run, this function will be generated',...
            '%%   point-mass: state = [x; y; z; vx; vy; vz; ax; ay; az]\n',...
            '%%   quadcopter: state = [x; y; z; vx; vy; vz; ax; ay; az; yaw; roll; Pitch];\n',...
            '%% control_mode:\n',...
            '%% TAKEOFF_TYPE = 2;\n',...
            '%% HOVER_TYPE = 3;\n',...
            '%% LAND_TYPE = 4;\n',...
            '%% POSITION_CONTROL_TYPE = 5;\n',...
            '%% VELOCITY_CONTROL_TYPE = 6;\n',...
            '%% VELOCITY_HORIZONTAL_CONTROL_TYPE = 7;\n']);

        %%% write switch-case %%%
        fprintf(file_id,'switch swarm_algorithm_type\n');
        for i = 1:length(field_names)
            fprintf(file_id,'\tcase ''%s''\n',field_names{i});
            fprintf(file_id,'\t\t[command_upper_s,control_mode_s] = %s_module_generate_desire(t, states, sample_time, sensor_data_s, map3d_struct, terrain, terrain_params, values);\n',field_names{i});
        end

        fprintf(file_id,'\totherwise\n');
        fprintf(file_id,'\t\tcommand_upper_s = [];\n\t\tcontrol_mode_s = [];\n');
        fprintf(file_id,'end');

        %%% write end%%%
        fprintf(file_id,'\n\nend');
        fclose(file_id);
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function write_subswarm_for_file(dir_name, file_name, subswarm_name, values, values_name)
        file_fullpath = strcat(dir_name,"/",file_name,".m");
        if ~exist(file_fullpath,'file')
            %%% write function name %%%
            file_id = fopen(file_fullpath,'w');
            fprintf(file_id,['function [command_upper_s,control_mode_s] =',...
                '%s_module_generate_desire(t, states, sample_time, sensor_data_s, map3d_struct, terrain, terrain_params, values)\n'],subswarm_name);

            fprintf(file_id,['%%%s_MODULE_GENERATE_DESIRE Generate the desired position and velocity\n',...
                '%% Automatically generated once by read_parameter_xml.m\n',...
                '%% This function will be called by swarms_module_generate_desire.m\n',...
                '%%   point-mass: state = [x; y; z; vx; vy; vz; ax; ay; az]\n',...
                '%%   quadcopter: state = [x; y; z; vx; vy; vz; ax; ay; az; yaw; roll; Pitch];\n',...
                '%% control_mode:\n',...
                '%% TAKEOFF_TYPE = 2;\n',...
                '%% HOVER_TYPE = 3;\n',...
                '%% LAND_TYPE = 4;\n',...
                '%% POSITION_CONTROL_TYPE = 5;\n',...
                '%% VELOCITY_CONTROL_TYPE = 6;\n',...
                '%% VELOCITY_HORIZONTAL_CONTROL_TYPE = 7;\n\n\n'],upper(subswarm_name));

            %%% Get parameters %%%%
            fprintf(file_id,'%% Parameters only be generated once by read_parameter_xml.m.\n');
            fprintf(file_id,'%% If you change the parameters of your swarm submodule, you need to\n');
            fprintf(file_id,'%% get parameters by %s_module_parameters()\n',subswarm_name);
            fprintf(file_id,'\n');
            fprintf(file_id,'%% The following operations are for multi-core parallel computing.\n');
            fprintf(file_id,'persistent fun_params\n');
            fprintf(file_id,'if isempty(fun_params)\n');
            fprintf(file_id,"\tfile_name_param = '%s_module_parameters';\n",subswarm_name);
            fprintf(file_id,'\t[~,str_core] = get_multi_core_value();\n');
            fprintf(file_id,'\tfun_params = str2func([file_name_param,str_core]);\n');
            fprintf(file_id,'end\n');
            fprintf(file_id,'\n');
            if ~isempty(values_name)
                fprintf(file_id,'[');
                for i = 1:length(values_name)
                    fprintf(file_id,'%s', values_name(i));
                    if i ~= length(values_name)
                        fprintf(file_id,',...\n');
                    end
                end
                fprintf(file_id,'] = fun_params();\n\n');
            else
                fprintf(file_id,'fun_params();\n\n');
            end
            %%% write %%%%
            fprintf(file_id,['%%\n',...
                'number = size(states,2);\n',...
                'command_upper_s = zeros(12,number);\n',...
                'command_upper_s(1:3,:) = states(1:3,:);\n',...
                'control_mode_s = uint8(zeros(1,number))+7;\n\n']);
            fprintf(file_id,['%% Traverse every agent\n',...
                'for id  = 1:number\n',...
                '\tpos_desired_id = [states(1:3,id);0];\n',...
                '\tpos_desired_id(3) = 1;\n',...
                '\tvel_desired_id = [mean(states(4:5,:), 2);0;0];\n',...
                '\tvel_desired_id(1:2) = 0.1*vel_desired_id(1:2)/norm(vel_desired_id(1:2));\n',...
                '\tacc_desired_id = [0;0;0;0];\n',...
                '\tcommand_upper_s(:,id) = [pos_desired_id;vel_desired_id;acc_desired_id];\n',...
                '\t%% Get sensor data\n',...
                '\tif ~isempty(sensor_data_s)\n',...
                '\t\trange_s = sensor_data_s(1,:,id);\n',...
                '\t\tpsi_s = sensor_data_s(2,:,id);\n',...
                '\t\tphi_s = sensor_data_s(3,:,id);\n',...
                '\tend\n']);

            fprintf(file_id,'end\n');
	% 
            %%% write end%%%
            fprintf(file_id,'\n\nend');
            fclose(file_id);
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function write_evaluation_module_one(dir_name, file_name, field_names, ctime_xml)
        file_fullpath = strcat(dir_name,"/",file_name,".m");
    
        % if exist(file_fullpath,"file")
        %     file_now = dir(file_fullpath);
        %     ctime_file = datenum(file_now.date);
        %     if ctime_file > ctime_xml
        %         return
        %     end
        % end
        %%% write function name %%%
        file_id = fopen(file_fullpath,'w');
        fprintf(file_id,['function values = evaluation_module_one(evaluation_metric_type, t, sample_time, states, map3d_faces, map3d_struct, terrain, terrain_params)\n']);

        fprintf(file_id,['%%EVALUATION_MODEL_ONE \n',...
            '%% Automatically generated by read_parameter_xml.m\n',...
            '%% Every time read_parameter_xml.m is run, this function will be generated\n']);

        %%% write switch-case %%%
        fprintf(file_id,'switch evaluation_metric_type\n');
        for i = 1:length(field_names)
            fprintf(file_id,'\tcase ''%s''\n',field_names{i});
            fprintf(file_id,'\t\tvalues = %s_module_one(t, sample_time, states, map3d_faces, map3d_struct, terrain, terrain_params);\n',field_names{i});
        end

        fprintf(file_id,'\totherwise\n');
        fprintf(file_id,'\t\tvalues = [];\n');
        fprintf(file_id,'end');

        %%% write end%%%
        fprintf(file_id,'\n\nend');
        fclose(file_id);
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function write_evaluation_module_average(dir_name, file_name, field_names, ctime_xml)
        file_fullpath = strcat(dir_name,"/",file_name,".m");
        
        % if exist(file_fullpath,"file")
        %     file_now = dir(file_fullpath);
        %     ctime_file = datenum(file_now.date);
        %     if ctime_file > ctime_xml
        %         return
        %     end
        % end
        %%% write function name %%%
        file_id = fopen(file_fullpath,'w');
        fprintf(file_id,['function values = evaluation_module_average(evaluation_metric_type, time_series, values_series)\n']);

        fprintf(file_id,['%%EVALUATION_MODEL_AVERAGE \n',...
            '%% Automatically generated by read_parameter_xml.m\n',...
            '%% Every time read_parameter_xml.m is run, this function will be generated\n']);

        %%% write switch-case %%%
        fprintf(file_id,'switch evaluation_metric_type\n');
        for i = 1:length(field_names)
            fprintf(file_id,'\tcase ''%s''\n',field_names{i});
            fprintf(file_id,'\t\tvalues = %s_module_average(time_series, values_series);\n',field_names{i});
        end

        fprintf(file_id,'\totherwise\n');
        fprintf(file_id,'\t\tvalues = [];\n');
        fprintf(file_id,'end');

        %%% write end%%%
        fprintf(file_id,'\n\nend');
        fclose(file_id);
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function write_subevaluation_one_for_file(dir_name, file_name, subeva_name, values, values_name)
        file_fullpath = strcat(dir_name,"/",file_name,".m");
        if ~exist(file_fullpath,'file')
            %%% write function name %%%
            file_id = fopen(file_fullpath,'w');
            fprintf(file_id,['function values =',...
                '%s_module_one(t, sample_time, states, map3d_faces, map3d_struct, terrain, terrain_params)\n'],subeva_name);

            fprintf(file_id,['%%%s_MODULE_ONE \n',...
                '%% Automatically generated once by read_parameter_xml.m\n',...
                '%% This function will be called by evaluation_module_one.m\n\n'],upper(subeva_name));

            %%% Get parameters %%%%
            fprintf(file_id,'%% Parameters only be generated once by read_parameter_xml.m.\n');
            fprintf(file_id,'%% If you change the parameters of your evaluation submodule, you need to\n');
            fprintf(file_id,'%% get parameters by %s_module_parameters()\n',subeva_name);
            fprintf(file_id,'\n');
            fprintf(file_id,'%% The following operations are for multi-core parallel computing.\n');
            fprintf(file_id,'persistent fun_params\n');
            fprintf(file_id,'if isempty(fun_params)\n');
            fprintf(file_id,"\tfile_name_param = '%s_module_parameters';\n",subeva_name);
            fprintf(file_id,'\t[~,str_core] = get_multi_core_value();\n');
            fprintf(file_id,'\tfun_params = str2func([file_name_param,str_core]);\n');
            fprintf(file_id,'end\n');
            if ~isempty(values_name)
                fprintf(file_id,'[');
                for i = 1:length(values_name)
                    fprintf(file_id,'%s', values_name(i));
                    if i ~= length(values_name)
                        fprintf(file_id,',...\n');
                    end
                end
                fprintf(file_id,'] = fun_params();\n\n');
            else
                fprintf(file_id,'fun_params();\n\n');
            end
            %%% write %%%%
            fprintf(file_id,['%%\n',...
                'speed = sqrt(sum(states(4:6,:).^2,1));\n',...
                'phi_corr = norm(mean(states(4:6,:)./speed,2));\n',...
                'phi_vel = mean(speed)/0.1;\n',...
                'values = [phi_corr;phi_vel];\n',...
                '\n\n']);

            %%% write end%%%
            fprintf(file_id,'\n\nend');
            fclose(file_id);
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function write_subevaluation_average_for_file(dir_name, file_name, subeva_name, values, values_name)
        file_fullpath = strcat(dir_name,"/",file_name,".m");
        if ~exist(file_fullpath,'file')
            %%% write function name %%%
            file_id = fopen(file_fullpath,'w');
            fprintf(file_id,['function values =',...
                '%s_module_average(time_series, values_series)\n'],subeva_name);

            fprintf(file_id,['%%%s_MODULE_AVERAGE \n',...
                '%% Automatically generated once by read_parameter_xml.m\n',...
                '%% This function will be called by evaluation_module_average.m\n\n'],upper(subeva_name));

            %%% Get parameters %%%%
            fprintf(file_id,'%% Parameters only be generated once by read_parameter_xml.m.\n');
            fprintf(file_id,'%% If you change the parameters of your evaluation submodule, you need to\n');
            fprintf(file_id,'%% get parameters by %s_module_parameters()\n',subeva_name);
            fprintf(file_id,'\n');
            fprintf(file_id,'%% The following operations are for multi-core parallel computing.\n');
            fprintf(file_id,'persistent fun_params\n');
            fprintf(file_id,'if isempty(fun_params)\n');
            fprintf(file_id,"\tfile_name_param = '%s_module_parameters';\n",subeva_name);
            fprintf(file_id,'\t[~,str_core] = get_multi_core_value();\n');
            fprintf(file_id,'\tfun_params = str2func(strcat(file_name_param,str_core));\n');
            fprintf(file_id,'end\n');
            if ~isempty(values_name)
                fprintf(file_id,'[');
                for i = 1:length(values_name)
                    fprintf(file_id,'%s', values_name(i));
                    if i ~= length(values_name)
                        fprintf(file_id,',...\n');
                    end
                end
                fprintf(file_id,'] = fun_params();\n\n');
            else
                fprintf(file_id,'fun_params();\n\n');
            end
            %%% write %%%%
            fprintf(file_id,['%%\n',...
                'phi_corr = mean(values_series(1,:));\n',...
                'phi_vel = mean(values_series(2,:));\n',...
                'F = phi_corr*phi_vel;\n',...
                'values = [F;phi_corr;phi_vel];\n',...
                '\n\n']);


            %%% write end%%%
            fprintf(file_id,'\n\nend');
            fclose(file_id);
        end
    end
end
