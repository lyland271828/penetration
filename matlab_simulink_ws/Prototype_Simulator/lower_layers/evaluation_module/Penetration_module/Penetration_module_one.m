function values =Penetration_module_one(t, sample_time, states, map3d_faces, map3d_struct, terrain, terrain_params)
%PENETRATION_MODULE_ONE
% Automatically generated once by read_parameter_xml.m
% This function will be called by evaluation_module_one.m

% Parameters only be generated once by read_parameter_xml.m.
% If you change the parameters of your evaluation submodule, you need to
% get parameters by Penetration_module_parameters()

% The following operations are for multi-core parallel computing.
persistent fun_params
if isempty(fun_params)
    file_name_param = 'Penetration_module_parameters';
    [~,str_core] = get_multi_core_value();
    fun_params = str2func([file_name_param,str_core]);
end
[uav_radius,...
v_flock,...
r_coll,...
rader_pos,...
rader_Rmax,...
rader_boold_min,...
target,...
num_radars] = fun_params();

% global pos_detected_x;
% global pos_detected_y;

number = size(states,2);




%
position = states(1:2,:);
velocity = states(4:5,:);

radar_models = "AN/MPQ-53";
uav_positions = states(1:3,:)';

[~, all_pos_detected] = detect_UAVs(terrain, num_radars, radar_models, rader_pos', uav_positions); %all_pos_detected为n行3列


uav_centers = states(1:3,:)';
detected_uav_ids = find_detected_uavs(all_pos_detected, uav_centers, uav_radius);  %为被探测到的无人机的id

if ~isempty(all_pos_detected)
    pos_detected_x = all_pos_detected(:,1);
    pos_detected_y = all_pos_detected(:,2);
else
    pos_detected_x = [];
    pos_detected_y = [];
end

save_pos_detected(pos_detected_x, pos_detected_y);


%%  计算血量
persistent blood;

if isempty(blood) || t==0
    blood = 100+zeros(number,1);
end


enemy_weapon = 'HELMTT';

% Define damage values for different weapons
damage_values = containers.Map({'HELMTT', 'HELTVD', 'IFPC-HEL'}, [100*sample_time/52.178, 100*sample_time/1.0073, 100*sample_time/0.00107]);

for id = 1:number
    if t > 0
        % Detect UAVs within radar range
        if any(ismember(id, detected_uav_ids))
            % Check if the UAV is in the detected list
            if isKey(damage_values, enemy_weapon)
                blood(id) = blood(id) - damage_values(enemy_weapon)*0.6;  %这里*0.7是为了血量好看，方便观察过程，以免一下掉完
            else
                error('Unknown enemy weapon: %s', enemy_weapon);
            end
        end
            % Ensure blood does not go below minimum threshold
        if blood(id) < rader_boold_min
            blood(id) = 0;

        end
    end

    if blood(id) < rader_boold_min
            detected_uav_ids(detected_uav_ids == id) = [];
    end
end


detected_uav_ids
%% 计算评估指标



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
%values = [phiCorr;phiVel;phiColl;phiWall;phigroup];

%success_rate = strcat(num2str(success_define(states,r_coll, map3d_faces, map3d_struct)),'%');

success_rate = (success_define(states,r_coll, map3d_faces, map3d_struct,target));

%Length = length(pos_detected_x);

values = [blood;success_rate;phiCorr;phiVel;phiColl;phiWall;phigroup];  %个数更改时，得到visual_module_draw_figures.m里面改，比如values_series(1:number,end);等values_series相关的数组。




end



%%%下面radar_positions为行向量



function [all_detected_UAVs, all_pos_detected] = detect_UAVs(terrain, num_radars, radar_models, radar_positions, uav_positions)
    % 加载高度地图
    height_map = terrain;

    % 创建雷达和无人机对象
    radars = cell(1, num_radars);
    for i = 1:num_radars
        radars{i} = Radar(radar_positions(i, :), radar_models{i});
    end

    % 创建障碍物（无人机）对象
    obstacles = [];
    for i = 1:size(uav_positions, 1)
        uav_x = uav_positions(i, 1);
        uav_y = uav_positions(i, 2);
        uav_z = uav_positions(i, 3);
        uav_RCS = 20; % 假设RCS为20
        obstacles = [obstacles; CylinderObstacle(uav_x, uav_y, uav_RCS, uav_z - uav_RCS, uav_RCS)];
    end
    obstacles = merge_obstacles(obstacles);

    % 模拟雷达扫描
    all_detected_UAVs = [];
    all_pos_detected = [];
    for i = 1:num_radars
        [detected_UAVs, pos_detected] = simulate_scan(radars{i}, height_map, obstacles);
        all_detected_UAVs =  detected_UAVs;
        all_pos_detected = pos_detected;
    end
end


function radar = Radar(origin, model)
    radar.origin = origin;
    radar.model = model;

    if strcmp(model, 'AN/MPQ-53')
        radar.max_accepted_obs = 70;
        radar.max_range = 160;
        radar.distance_resolution = 1.5;
        radar.angle_resolution = 0.03;
        radar.max_angle = 2 * pi / 3;
        radar.num_rays = round(radar.max_angle / radar.angle_resolution);
        radar.num_vertical = 20;
        radar.mounting_angle = 0;
    elseif strcmp(model, '96L6E')
        radar.max_accepted_obs = 70;
        radar.max_range = 177;
        radar.distance_resolution = 1.5;
        radar.angle_resolution = 0.03;  
        radar.max_angle = 2 * pi / 3;
        radar.num_rays = round(radar.max_angle / radar.angle_resolution);
        radar.num_vertical = 20;
        radar.mounting_angle = 0;
    else
        error('Unknown radar model');
    end
end


function [detected_UAVs, pos_detected] = simulate_scan(radar, height_map, obstacles)
    angles_horizontal = linspace(radar.mounting_angle, radar.mounting_angle + radar.max_angle, radar.num_rays);
    angles_vertical = linspace(-pi / 6, pi / 6, radar.num_vertical);
    x_origin = radar.origin(1);
    y_origin = radar.origin(2);
    z_origin = radar.origin(3);

    [horizontal_grid, vertical_grid] = meshgrid(angles_horizontal, angles_vertical);
    x_end = x_origin + radar.max_range * cos(vertical_grid) .* cos(horizontal_grid);
    y_end = y_origin + radar.max_range * cos(vertical_grid) .* sin(horizontal_grid);
    z_end = z_origin + radar.max_range * sin(vertical_grid);

    num_points = round(radar.max_range / radar.distance_resolution);
    x_line = linspace(0, 1, num_points);
    
    x_paths = x_origin + (x_end(:) - x_origin) * x_line;
    y_paths = y_origin + (y_end(:) - y_origin) * x_line;
    z_paths = z_origin + (z_end(:) - z_origin) * x_line;

    detected_UAVs = [];
    pos_detected = [];

    % 将所有路径点转换为线性索引
    xi = min(max(round(x_paths), 1), size(height_map, 2));
    yi = min(max(round(y_paths), 1), size(height_map, 1));
    height_at_points = height_map(sub2ind(size(height_map), yi, xi));

    % 检查地形碰撞
    terrain_collision = z_paths <= (height_at_points-3000);
    
    % 检查UAV碰撞
    uav_collision = false(size(x_paths));
    for i = 1:length(obstacles)
        obs = obstacles(i);
        distance_to_obs_center = sqrt((x_paths - obs.center_x).^2 + (y_paths - obs.center_y).^2);
        in_radius = distance_to_obs_center <= obs.radius;
        in_height = z_paths >= obs.bottom_z & z_paths <= obs.bottom_z + obs.height;
        uav_collision = uav_collision | (in_radius & in_height);
    end
    
    % 检查每条路径
    for i = 1:numel(x_end)
        path_terrain_collision = find(terrain_collision(i, :), 1);
        path_uav_collision = find(uav_collision(i, :), 1);
        
        if ~isempty(path_uav_collision) && (isempty(path_terrain_collision) || path_uav_collision < path_terrain_collision)
            detected_UAVs = [detected_UAVs, i];
            pos_detected = [pos_detected; x_paths(i, path_uav_collision), y_paths(i, path_uav_collision), z_paths(i, path_uav_collision)];
        elseif ~isempty(path_terrain_collision)
            % 路径停止在地形碰撞点，不需要继续处理
            continue;
        end
    end

    detected_UAVs = unique(detected_UAVs); % 保证唯一性
end



function detected_uav_ids = find_detected_uavs(pos_detected, uav_centers, uav_radius)
    % 获取被探测到的点和无人机中心的数量
    num_detected_points = size(pos_detected, 1);
    num_uavs = size(uav_centers, 1);

    % 扩展无人机中心的维度
    expanded_uav_centers = reshape(uav_centers, [1, num_uavs, 3]);
    expanded_uav_centers = repmat(expanded_uav_centers, [num_detected_points, 1, 1]);

    % 扩展被探测到的点的维度
    expanded_pos_detected = reshape(pos_detected, [num_detected_points, 1, 3]);
    expanded_pos_detected = repmat(expanded_pos_detected, [1, num_uavs, 1]);

    % 计算所有点与所有无人机中心的距离
    distances = sqrt(sum((expanded_pos_detected - expanded_uav_centers).^2, 3));

    % 找出所有在无人机半径范围内的无人机
    [detected_points_idx, detected_uavs_idx] = find(distances <= uav_radius);

    % 获取检测到的无人机ID
    detected_uav_ids = unique(detected_uavs_idx);
end




function status = check_collision(x, y, z, height_map, obstacles, max_accepted_obs)
    % [rows, cols] = size(height_map);
    % xi = min(max(round(x), 1), cols);
    % yi = min(max(round(y), 1), rows);
    % if z <= height_map(yi, xi)
    %     status = 'terrain_occlusion';
    %     return;
    % end
    for i = 1:length(obstacles)
        obs = obstacles(i);
        if sqrt((x - obs.center_x)^2 + (y - obs.center_y)^2) <= obs.radius && ...
                obs.bottom_z <= z && z <= obs.bottom_z + obs.height && obs.radius < max_accepted_obs
            status = 'UAV_found';
            return;
        end
    end
    status = 'Nothing';
end

% function status = check_collision(x, y, z, height_map, obstacles, max_accepted_obs)
%     [rows, cols] = size(height_map);
%     xi = min(max(round(x), 1), cols);
%     yi = min(max(round(y), 1), rows);
% 
%     if z <= height_map(yi, xi)
%         status = 'terrain_occlusion';
%         return;
%     end
% 
%     for i = 1:length(obstacles)
%         obs = obstacles(i);
%         if sqrt((x - obs.center_x)^2 + (y - obs.center_y)^2) <= obs.radius && ...
%            obs.bottom_z <= z && z <= obs.bottom_z + obs.height && obs.radius < max_accepted_obs
%             status = 'UAV_found';
%             return;
%         end
%     end
% 
%     status = 'Nothing';
% end



function obstacles = CylinderObstacle(center_x, center_y, radius, bottom_z, height)
    obstacles.center_x = center_x;
    obstacles.center_y = center_y;
    obstacles.radius = radius;
    obstacles.bottom_z = bottom_z;
    obstacles.height = height;
end

function merged = merge_obstacles(obstacles)
    merged = [];
    for i = 1:length(obstacles)
        new_obs = obstacles(i);
        merged_flag = false;
        for j = 1:length(merged)
            existing_obs = merged(j);
            if sqrt((new_obs.center_x - existing_obs.center_x)^2 + ...
                    (new_obs.center_y - existing_obs.center_y)^2) < (new_obs.radius + existing_obs.radius)
                existing_obs.radius = new_obs.radius + existing_obs.radius;
                existing_obs.height = new_obs.height + existing_obs.height;
                existing_obs.bottom_z = (new_obs.bottom_z * new_obs.radius + existing_obs.bottom_z * existing_obs.radius) / ...
                                        (existing_obs.radius + new_obs.radius);
                existing_obs.center_x = (new_obs.center_x * new_obs.radius + existing_obs.center_x * existing_obs.radius) / ...
                                        (existing_obs.radius + new_obs.radius);
                existing_obs.center_y = (new_obs.center_y * new_obs.radius + existing_obs.center_y * existing_obs.radius) / ...
                                        (existing_obs.radius + new_obs.radius);
                merged_flag = true;
                break;
            end
        end
        if ~merged_flag
            merged = [merged; new_obs];
        end
    end
end



