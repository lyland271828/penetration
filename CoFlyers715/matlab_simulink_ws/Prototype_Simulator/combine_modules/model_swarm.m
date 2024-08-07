function performances_time_average = model_swarm(parameters_gui,...
    parameters_op, parameters_bp, re, mode_simulation, xml_name, fig_app)
%MODEL_SWARM Combines the modules in lower layer into one module
%INPUT:
%   mode_simulation: =0 when rapid prototyping, =1 when auto-tuning, =2
%   when batch processing
%   parameters_gui: parameters from gui.
%   parameters_op: parameters to be optimized.
%   parameters_bp: parameters for batch processing.
%   re: mask the number of repetitions of the simulation
%   Non-empty parameter set in upper layer will modify the corresponding parameters in lower layer.
%OUTPUT:
%   performances_time_average: the time averages of performances


%% Initialization
%%% get parameters from the XML parameter file %%%
[states,dstates,...
    map3d_faces, map3d_struct, model_stls,...
    terrain,terrain_params] = initialize_parameters_states(parameters_gui,parameters_op, ...
    parameters_bp, mode_simulation, xml_name);

%%% Get setting from inputs %%%
[number,...
time_max,...
sample_time_motion,...
sample_time_control_upper,...
sample_time_control_bottom,...
activate_save_states,...
time_interval_save,...
motion_model_type,...
swarm_algorithm_type,...
evaluation_metric_type] = setting_parameters();

sample_time_control_upper = max(sample_time_control_upper,sample_time_motion);
sample_time_control_bottom = max(sample_time_control_bottom,sample_time_motion);
rate_upper   = ceil(sample_time_control_upper/sample_time_motion);
rate_bottom = ceil(sample_time_control_bottom/sample_time_motion);

% step size of saving trajectories
interval_state_save = max(floor(time_interval_save/sample_time_motion),1); 

%%% file name %%%
time_now_string = char(datetime('now','TimeZone','local','Format','y_M_d_H_m_s_SSS'));
data_save_dir_name = get_dir_name_from_mode(mode_simulation);

%%% Initialize states %%%
count = 1;
t = 0;
states_ob = motion_module_observation(states,dstates,motion_model_type);
states_m = noise_module_add_noise(states_ob);
if sensor_env_module_get_activate()
    sensor_data_s = sensor_env_module_get_data_all(states_ob, motion_model_type,map3d_faces,terrain,terrain_params);
else
    sensor_data_s = [];
end
values = evaluation_module_one(evaluation_metric_type, t, sample_time_control_upper, states, map3d_faces, map3d_struct, terrain, terrain_params);
[commands_upper,control_mode_s] =...
    swarm_module_generate_desire(swarm_algorithm_type, t,states_m, sample_time_control_upper, sensor_data_s, map3d_struct, terrain, terrain_params, values);
commands_bottom = motion_module_bottom_control(t, states, commands_upper,...
    control_mode_s, motion_model_type, sample_time_control_bottom);

%%% Initialize data series %%%
count_data_save = 1;
iter_max = floor(time_max/sample_time_motion)+1;
num_data_save = ceil(iter_max/interval_state_save);
states_ob_series = zeros([size(states_ob),num_data_save]);
states_ob_series(:,:,count_data_save) = states_ob;
time_series = (0:iter_max-1)*sample_time_motion;

count_upper = 1;
values_series = zeros([length(values), floor(time_max/sample_time_control_upper)+1]);
values_series(:,count_upper) = values;

%%% Initialize figure, axes and video %%%
if visual_module_get_activate(sample_time_motion, t, true)
    % Init figure
    if ~isempty(fig_app)
        fig = fig_app;
    else
        fig = figure;
    end

    % Refresh axes in the figure
    flag_stage = 0;
    visual_module_draw_figures(states, time_series(1), states_ob_series(:,:,1), time_series(1), values_series(:,1),...
        map3d_faces, map3d_struct, model_stls, terrain, terrain_params, mode_simulation, flag_stage, motion_model_type, fig);

    % Mouse click event of the left axis
    if visual_module_get_activate_BD()
        [map3d_faces,map3d_struct] = visual_module_buttondown(t, fig, map3d_faces, map3d_struct);
    end

end

%% Iteraction

%%% loop %%%
for t = sample_time_motion:sample_time_motion:time_max
    
    % Listen for the event of stopping simulation in app
    if ~isempty(fig_app)
        if isfield(fig_app.UserData, 'flag_stop')
            if fig_app.UserData.flag_stop
                break
            end
        end
    end

    % Next iteration
    count = count + 1;
    
    % Observation
    states_ob = motion_module_observation(states,dstates,motion_model_type);

    % Add noise to states_ob
    states_m = noise_module_add_noise(states_ob);

    % Upper control
    if mod(count,rate_upper) == 0
        count_upper = count_upper + 1;
        % Get Sensor data from map3d_faces
        if sensor_env_module_get_activate()
            sensor_data_s = sensor_env_module_get_data_all(states_ob, motion_model_type,map3d_faces,terrain,terrain_params);
        else
            sensor_data_s = [];
        end

        % Get desired position and velocity from flocking rules
        [commands_upper,control_mode_s] =...
            swarm_module_generate_desire(swarm_algorithm_type, t,states_m, sample_time_control_upper, sensor_data_s, map3d_struct, terrain, terrain_params, values);

        % Calculate the performance of current frame
        [values] = evaluation_module_one(evaluation_metric_type, t, sample_time_control_upper, states_ob,map3d_faces, map3d_struct, terrain, terrain_params);
        values_series(:,count_upper) = values;
    end
    
    % Bottom control
    if mod(count,rate_bottom) == 0
        commands_bottom = motion_module_bottom_control(t, states, commands_upper,...
            control_mode_s, motion_model_type, sample_time_control_bottom);
    end

    % Dynamics and kinematics simulation 
    [states, dstates] = motion_module_update_dynamics(t, states,commands_bottom,...
        motion_model_type, sample_time_motion);
    
    % Updata map
    [map3d_faces,map3d_struct] = map_module_update_map3d(t,sample_time_motion,map3d_faces,map3d_struct);
    if map_module_periodic_boundary_activate()
        states = map_module_periodic_boundary(states);
    end
    
    % Mouse click event of the left axis
    if visual_module_get_activate_BD()
        [map3d_faces,map3d_struct] = visual_module_buttondown(t, fig, map3d_faces,map3d_struct);
    end

    % Save data
    if mod(count-1,interval_state_save) == 0
        count_data_save = count_data_save + 1;
        states_ob_series(:,:,count_data_save)    = states_ob;
    end
    
    % Plot 
    if visual_module_get_activate(sample_time_motion, t, false)
        flag_stage = 1;
        visual_module_draw_figures(states_ob, time_series(1:interval_state_save:count), states_ob_series(:,:,1:count_data_save),...
            linspace(0,time_series(count),count_upper), values_series(:,1:count_upper),...
            map3d_faces, map3d_struct, model_stls, terrain, terrain_params, mode_simulation, flag_stage, motion_model_type, fig);
    end
end
%==========================================================
if mod(count-1,interval_state_save) ~= 0
        states_ob_series(:,:,count_data_save)    = states_ob;
end
% Calculate the performance of the whole process
performances_time_average = evaluation_module_average(evaluation_metric_type, linspace(0,time_series(count),count_upper), values_series);

% Plot values, final trajectory and close Video
if visual_module_get_activate(sample_time_motion, t, true)
    flag_stage = 2;
    visual_module_draw_figures(states_ob, time_series(1:interval_state_save:end),...
        states_ob_series, linspace(0,time_series(count),count_upper), values_series,...
        map3d_faces, map3d_struct, model_stls, terrain, terrain_params, mode_simulation, flag_stage, motion_model_type, fig);
end

% Save data
if activate_save_states
    t = getCurrentTask();
    id = 1;
    if ~isempty(t)
        id = t.ID;
    end
    save([data_save_dir_name,'/mats/data_',time_now_string,'_w',num2str(id),'_re',num2str(re),'.mat']);
end


end