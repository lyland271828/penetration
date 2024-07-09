function [command_upper_s,control_mode_s] =olfati_saber_module_generate_desire(t, states, sample_time, sensor_data_s, map3d_struct, terrain, terrain_params, values)
%OLFATI_SABER_MODULE_GENERATE_DESIRE Generate the desired position and velocity
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
% get parameters by olfati_saber_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
	file_name_param = 'olfati_saber_module_parameters';
	[~,str_core] = get_multi_core_value();
	fun_params = str2func([file_name_param,str_core]);
end

[v_flock,...
leader,...
swarm_r_coll,...
sim_dt,...
swarm_a,...
swarm_b,...
swarm_c,...
swarm_d_ref,...
nb_neig,...
swarm_delta,...
swarm_r,...
swarm_r0,...
r_shill_0,...
v_shill,...
p_shill,...
a_shill,...
obR,...
v_max,...
dim,...
height,...
dr_shill,...
alpha_rejob,...
alpha_atttar,...
pos_shill,...
vel_shill,...
rader_boold_min] = fun_params();

%
number = size(states,2);
command_upper_s = zeros(12,number);
command_upper_s(1:3,:) = states(1:3,:);
control_mode_s = uint8(zeros(1,number))+7;

% canyon_snow_map
if states(1,1) < -140
    target = [-70;-11;120];
else
    target=[300;-10;180];
end


    % Swarm parameters
    r_agent = swarm_r_coll;

    % Simulation parameters
    dt = sample_time;

    %% Compute useful functions    
    % Weight function for computing the motion planning acceleration
    psi = @(z) ((swarm_p + swarm_b) * (sqrt(1 + (z - swarm_d_ref + swarm_c)^2) - sqrt(1 + swarm_c^2)) + ...
        (swarm_a - swarm_b) * (z - swarm_d_ref)) / 2;
    psi_der = @(z) (swarm_a + swarm_b) / 2 * (z - swarm_d_ref + swarm_c) ./ sqrt(1 + (z - swarm_d_ref + swarm_c)^2) + ...
        (swarm_a - swarm_b) / 2;
    sigma_beta = @(z) z/sqrt(1 + z^2) - 1;

    % Generalization of adjacency coefficients for computing the motion planning acceleration
    rho = @rho_f;
    rho_der = @rho_der_f;

    nb_neig = 6;                % Number of neighbours

    % Force defining the attraction/repulsion as function of the distance
    phi = @(z) rho(z, swarm_delta, swarm_r, nb_neig)*psi_der(z);

    state = states;
    pos = state(1:3,:);
    vel = state(4:6,:);
    
    %% Initialize variables
    
    nb_agents = size(state,2);
    M = zeros(nb_agents, nb_agents);    % Neighborhood matrix
    D = zeros(nb_agents, nb_agents);    % Distance matrix
    acc_pm = zeros(3, nb_agents);       % Position matching acceleration
    acc_vm = zeros(3, nb_agents);       % Velocity matching acceleration
    acc_lm = zeros(3, nb_agents);       % Leader matching
    acc_wall = zeros(3, nb_agents);     % Arena repulsion acceleration
    acc_obs = zeros(3, nb_agents);      % Obstacle repulsion acceleration
    acc_command = zeros(3, nb_agents);  % Calculate the commanded acceleration
    vel_command = zeros(3, nb_agents);  % Calculate the commanded velocity
    acce_commands = zeros(3,nb_agents);

    nb_agent_collisions = 0;    % Nb of collisions among agents
    nb_obs_collisions = 0;      % Nb of collisions against obstacles
    min_dist_obs = 0.4;          % Init minimum distance to obstacles

    
    %% Compute velocity commands for every agent
    
    for agent = 1:nb_agents

        if agent == 1
            % For the first agent,we give it velocity instructions instead of acceleration instructions
            vel_command(:,1) = [v_flock;0;0];
            % self.drones(1).command = u;
            % acce_commands(:,1) = u;
        else
            %% Find neighbors
            % Compute agent-agent distance matrix
            p_rel = pos - pos(:, agent);
            dist = sqrt(sum((p_rel.^2), 1));
            D(agent, :) = dist;

            % Count collisions
            nb_agent_collisions = nb_agent_collisions + sum(dist < 2 * r_agent) - 1;

            % Define neighbors list

            neig_lis = (1:nb_agents)';  % All agents' id: [1 2 3 ...]
            [~, idx] = sort(dist);      % sort distance and get id
            idx = idx(1:nb_neig + 1);   % the id of nearest K+1 drones except itself
            neig_lis = neig_lis(idx);   % Transpose: neig_lis = idx'
            if ~ismember(1, idx)
                neig_lis(end) = 1;
            end
            neig_lis(1) = []; % Remove itself

            % Adjacency matrix (asymmetric in case of limited fov)
            M(agent, neig_lis) = 1;


            %% Compute different contributions

            % relative velocity and relative position (unit)
            vel_rel = vel - vel(:, agent);
            pos_rel_u = p_rel ./ dist;

            % Leader matching
            c_1_r = 0.1;
            acc_lm(:, agent) = c_1_r * p_rel(:, 1); % position matching
            c_2_r = 1;
            acc_lm(:, agent) = acc_lm(:, agent) + c_2_r * vel_rel(:, 1); % velocity matching

            for agent2 = neig_lis'
                % Position matching
                acc_pm(:, agent) = acc_pm(:, agent) + phi(dist(agent2)) * pos_rel_u(:, agent2);

                % Velocity matching
                acc_vm(:, agent) = acc_vm(:, agent) + rho(dist(agent2), swarm_delta, swarm_r, nb_neig) * vel_rel(:,agent2);
            end


            %% Sum contributions
            acc_command(:, agent) = acc_lm(:, agent) + acc_pm(:, agent) + acc_vm(:, agent) + acc_obs(:, agent);

            % Integrate acceleration to get velocity
            vel_command(:, agent) = vel(:, agent) + acc_command(:, agent) * dt;

        end


        %% attracted by target
        posId = states(1:3,agent); % i智能体位置
        posi_tar = posId - target;
        disi_tar = sqrt(sum(posi_tar.^2,1));

        % attID_tar = -alpha_atttar * posi_tar .* sqrt(disi_tar);
        attID_tar = -alpha_atttar * sqrt(disi_tar) .* posi_tar ./ disi_tar;

        %%%%%Avoidance obstacles term%%%%%
        state_i = states(:,agent);
        [obs,vel_shill] = map_module_generate_shill_agent_by_terrain(state_i,terrain,terrain_params,obR);
        rejID_ob = zeros(3,1);
        if ~isempty(obs)
            posi_ob = kron(ones(1,size(obs,2)),posId) - obs; % i智能体与障碍物之间的相对位置
            disi_ob = sqrt(sum(posi_ob.^2,1)); % i智能体到所有障碍物之间的距离

            lessThanobR = disi_ob < obR;
            neighborIndexobR = find(lessThanobR); % 与i智能体距离相近的障碍物索引j(小于该值防止防撞)

            if ~isempty(neighborIndexobR)
                dis_rejiob = disi_ob(neighborIndexobR);
                pos_rejiob = posi_ob(:,neighborIndexobR);
                rejID_ob = alpha_rejob * sum(pos_rejiob ./ (dis_rejiob-5),2);
            end
        end




        vel_command(1:dim,agent) = vel_command(1:dim,agent) + rejID_ob(1:dim) + attID_tar(1:dim);
    end
    
%%血量坠毁计算




for id=1:number
    blood = values(1:number,end);
    blood_id = blood(id);
    if blood_id < rader_boold_min
        if states(3,id)>0
            vel_command(1:3,id) = [0;0;-5];

        else
            vel_command(1:3,id) = [0;0;0];
        end
    end
end

    %% Compute collisions and bound velocities and accelerations

    command_upper_s(3,:) = height;
    command_upper_s(5:7,:) = vel_command;

end

function y = rho_f(z, delta, r, k)
    if z < delta * r
        y = 1;
        return;
    elseif z < r
        y = (1/2^k) * (1 + cos(pi * (z / r - delta) ./ (1 - delta)))^k;
        return;
    else
        y = 0;
    end
end

function y = rho_der_f(z, delta, r, k)
    if z < delta * r
        y = 0;
        return;
    elseif z < r
        arg = pi * (z / r - delta) ./ (1 - delta);
        y = -pi / (1 - delta) * k / (2^k) * (1 + cos(arg))^(k - 1) * (sin(arg));
        return;
    else
        y = 0;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function a_arena = repulsion_cubic_arena(pos_drone, pos_walls, wall_width, const_rep)
    a_arena = zeros(3, 1);
    for axis = 1:3
        x_drone = pos_drone(axis);
        % Repulsion from left wall
        x_arena = pos_walls(1,axis);

        if x_drone < x_arena
            a_arena(axis) = a_arena(axis) + const_rep;
        elseif x_drone > x_arena && x_drone < x_arena + wall_width
            a_arena(axis) = a_arena(axis) + const_rep * 0.5 * (sin((pi / wall_width) * (x_drone + x_arena) + pi / 2) + 1);
        end

        % Repulsion from right wall
        x_arena = pos_walls(8,axis);

        if x_drone > x_arena - wall_width && x_drone < x_arena
            a_arena(axis) = a_arena(axis) - const_rep * 0.5 * (sin((pi / wall_width) * (x_drone - x_arena - wall_width) - pi / 2) + 1);
        elseif x_drone > x_arena
            a_arena(axis) = a_arena(axis) - const_rep;
        end
    end
end